

#include "geometry.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <iostream>

int emap_width, emap_height;
struct Material
{
    Material(const float &r, const Vec4f &a, const Vec3f &color, const float &spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : albedo(1, 0, 0, 0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

Vec3f reflect(const Vec3f &I, const Vec3f &N)
{
    return I - N * 2.f * (I * N);
}

Vec3f refract(const Vec3f &I, const Vec3f &N, const float &refractive_index)
{ // Snell's law
    /*
    n1    sin02
    –  =  –––––
    n2    sin01
    */
    float cosi = -std::max(-1.f, std::min(1.f, I * N));
    float etai = 1, etat = refractive_index;
    Vec3f n = N;
    if (cosi < 0)
    {
        // if the ray is inside the object, swap the indices and invert the normal to get the correct result
        cosi = -cosi;
        std::swap(etai, etat);
        n = -N;
    }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? Vec3f(0, 0, 0) : I * eta + n * (eta * cosi - sqrtf(k));
}

struct Light
{
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

struct Sphere
{

    Vec3f center; //creates a 3d position
    float radius;
    Material material;

    Sphere(const Vec3f &c, const float &r, const Material &m) : center(c), radius(r), material(m) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const
    {

        //center is the cente3r of the sphere
        //origin is the position of the cam
        //dir is the screen

        Vec3f L = center - orig; //vector from cam to center of the sphere
        float tca = L * dir;     // dot product

        /*
                                  tca
                                   |
        origin of ray  --> *_______ˇ_______> 
                            #              |
                              #            |   
                                #          |
                                  #        |
                            L       #      |   
                                      #    |
                                         # |
                                           *  <--center of circle
        
        */

        //d2 is the square of the length of the center of the circle to the vector
        //that means if d2 < radius * radius the ray intersects the circle
        //if d2 = radius * radius the ray is at the edge of the circle
        //if d2 > radius * radius the ray does not hit the circle

        float d2 = L * L - tca * tca;
        if (d2 > radius * radius)
            return false;

        float thc = sqrtf(radius * radius - d2);
        //thc is the length of the of the vector from point of intersection to d2
        /*
                                  thc
                                   |
point of intersection  --> *_______ˇ_______
                            #              |
                              #            |   
                                #          |
                                  #        |√d2
                         radius     #      |   
                                      #    |
                                         # |
                                           *  <--center of circle
        
        */
        t0 = tca - thc;
        float t1 = tca + thc;
        //t0 and t1 are the points of intersections
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        return true;
    }
};

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material)

{
    float spheres_dist = std::numeric_limits<float>::max();

    for (size_t i = 0; i < spheres.size(); i++)
    {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist)
        {
            spheres_dist = dist_i;
            hit = orig + dir * dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }
    float checkerboard_dist = std::numeric_limits<float>::max();

    if (fabs(dir.y) > 1e-3)
    {
        float d = -(orig.y + 5) / dir.y;
        Vec3f pt = orig + dir * d;
        if (d > 0 && fabs(pt.x) < 10 && pt.z < -10 && pt.z > -30 && d < spheres_dist)
        {
            checkerboard_dist = d;
            hit = pt;
            N = Vec3f(0, 1, 0);
            material.diffuse_color = (int(.5 * hit.x + 1000) + int(.5 * hit.z)) & 1 ? Vec3f(.3, .3, .3) : Vec3f(.3, .2, .1);
        }
    }
    return std::min(spheres_dist, checkerboard_dist) < 1000;
};

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, const std::vector<Vec3f> &emap, size_t depth = 0)
{

    Vec3f point, N;
    float sphere_dist = std::numeric_limits<float>::max();
    Material material;

    if (depth > 4 || !scene_intersect(orig, dir, spheres, point, N, material))
    {
        //emap[i + 2000 + ((j + 1500) * (emap_width))] just puts the background image on screen

        return Vec3f(1, 1, 1); // background
    }

    Vec3f reflect_dir = reflect(dir, N);
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f reflect_orig = reflect_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3; // offset the original point to avoid occlusion by the object itself
    Vec3f refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;

    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, emap, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, emap, depth + 1);

    float diffuse_light_intensity = 0,
          specular_light_intensity = 0;

    for (size_t i = 0; i < lights.size(); i++)
    {
        Vec3f light_dir = (lights[i].position - point).normalize();
        diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir * N);
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N) * dir), material.specular_exponent) * lights[i].intensity;
    }

    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.) * specular_light_intensity * material.albedo[1] + reflect_color * material.albedo[2] + refract_color * material.albedo[3];
    ;
};
void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights, const std::vector<Vec3f> &emap)
{
    const int width = 1024;
    const int height = 768;
    const float fov = M_PI / 3.;
    std::vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
    for (int j = height - 1; j >= 0; j--)
    { // actual rendering loop
        for (int i = width - 1; i >= 0; i--)
        {
            float dir_x = (i + 0.5) - width / 3.;
            float dir_y = -(j + 0.5) + height / 3.; // this flips the image at the same time
            float dir_z = -height / (2. * tan(fov / 2.));
            framebuffer[i + j * width] = cast_ray(Vec3f(0, 0, 1), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights, emap);
        }
    }
    std::vector<unsigned char> pixmap(height * width * 3);
    for (size_t i = 0; i < (height * width - 1); ++i) //each pixel
    {
        Vec3f &c = framebuffer[i]; //vector frame buffer size of screen
                                   //c = address of framebuffer[i]
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max > 1)
            c = c * (1. / max);
        for (size_t j = 0; j < 3; j++)
        {
            pixmap[i * 3 + j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    stbi_write_jpg("out.jpg", width, height, 3, pixmap.data(), 100);
}

int main()
{
    int n = -1;
    unsigned char *pixmap = stbi_load("./emap.jpg", &emap_width, &emap_height, &n, 0);
    if (!pixmap || 3 != n)
    {
        std::cerr << "Can't load env map" << std::endl;
        return -1;
    }
    std::vector<Vec3f> emap;
    emap = std::vector<Vec3f>(emap_height * emap_width);
    std::cout << " width " << emap_width << std::endl;
    std::cout << " height " << emap_height << std::endl;
    std::cout << " size " << emap.size() << std::endl;

    std::cout << " k " << emap_width - 1 << std::endl;
    std::cout << " l " << emap_height - 1 << std::endl;
    std::cout << " pos " << (emap_width - 1) + (emap_height - 1) * emap_width << std::endl;
    for (int l = emap_height - 1; l >= 0; l--)
    {
        for (int k = emap_width - 1; k >= 0; k--)
        {
            emap[k + l * emap_width] = Vec3f(pixmap[(k + l * emap_width) * 3 + 0], pixmap[(k + l * emap_width) * 3 + 1], pixmap[(k + l * emap_width) * 3 + 2]) * (1 / 255.);
        }
    }
    stbi_image_free(pixmap);
    //material var1 =  var2 =color(RGB) var3 =
    Material ivory(1, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3), 50.);
    Material brown(1, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.44, 0.17, 0.07), 100.);
    Material red_rubber(1, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.);
    Material mirror(1, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);
    Material glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125.);

    std::vector<Sphere> spheres;

    Sphere sphere(Vec3f(3, 0, -10), 2, mirror); //create sphere

    Sphere sphere2(Vec3f(0, 0, -15), 1.5, glass); //create sphere

    Sphere sphere3(Vec3f(0, 2, -20), 1.3, red_rubber); //create sphere

    Sphere sphere4(Vec3f(-3, -1, -10), 1.3, brown); //create sphere

    Sphere sphere5(Vec3f(-0, -3, -12), 0.8, ivory); //create sphere

    spheres.push_back(sphere);
    spheres.push_back(sphere2);
    spheres.push_back(sphere3);
    spheres.push_back(sphere4);
    spheres.push_back(sphere5);

    std::vector<Light> lights;

    lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
    lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f(30, 20, 30), 1.7));

    render(spheres, lights, emap);

    return 0;
}
