#include "geometry.h"
#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <iostream>

struct Material
{
    Material(const Vec3f &color) : diffuse_color(color) {}
    Material() : diffuse_color() {}
    Vec3f diffuse_color;
};

struct Light
{
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};
struct Square
{
    Vec3f center;
    float length;
    Material material;
    Square(const Vec3f &c, const float &l, const Material &m) : center(c), length(l), material(m) {}
    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const
    {
        Vec3f L = center - orig;
        float tca = L * dir;
        float d2 = L * L - tca * tca;
        if (d2 > length * length)
            return false;
        return false;
    };
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

        //        std::cout << "tca ===  x " << tca << "\n";
        float d2 = L * L - tca * tca;
        if (d2 > radius * radius)
            return false;
        float thc = sqrtf(radius * radius - d2);
        t0 = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        return true;
    }
};

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Square> squares, Vec3f &hit, Vec3f &N, Material &material)
{
    float spheres_dist = std::numeric_limits<float>::max();
    float squares_dist = std::numeric_limits<float>::max();
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

    return spheres_dist < 1000;
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Square> squares, const std::vector<Light> &lights)
{

    Vec3f point, N;
    float sphere_dist = std::numeric_limits<float>::max();
    Material material;

    if (!scene_intersect(orig, dir, spheres, squares, point, N, material))
    {
        return Vec3f(0.2, 0.7, 0.8); // background color
    }
    float diffuse_light_intensity = 0;
    for (size_t i = 0; i < lights.size(); i++)
    {
        Vec3f light_dir = (lights[i].position - point).normalize();
        diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir * N);
    }
    return material.diffuse_color * diffuse_light_intensity;
}
void render(const std::vector<Sphere> &spheres, const std::vector<Square> &squares, const std::vector<Light> &lights)
{
    const int width = 1024;
    const int height = 768;
    const int fov = M_PI / 1.75;

    std::vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
    for (size_t j = 0; j < height; j++)
    {
        for (size_t i = 0; i < width; i++)
        {

            float x = (2 * (i + 0.5) / (float)width - 1) * tan(fov / 2.) * width / (float)height;
            float y = -(2 * (j + 0.5) / (float)height - 1) * tan(fov / 2.);

            Vec3f dir = Vec3f(x, y, -2).normalize();

            framebuffer[i + j * width] = cast_ray(Vec3f(0, 0, 0), dir, spheres, squares, lights);
        }
    }

    std::ofstream ofs; // save the framebuffer to file
    ofs.open("./out.ppm");
    ofs << "P6\n"
        << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height * width; ++i)
    {
        for (size_t j = 0; j < 3; j++)
        {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}

int main()
{

    Material ivory(Vec3f(0.85, 0.85, 0.85));
    Material ivory2(Vec3f(0.86, 0.86, 0.86));
    Material ivory3(Vec3f(0.83, 0.83, 0.83));
    Material red_rubber(Vec3f(0.3, 0.1, 0.1));
    std::vector<Sphere> spheres;
    std::vector<Square> squares;

    Sphere sphere(Vec3f(3, 0, -15), 2, ivory); //create sphere

    Sphere sphere2(Vec3f(0, 0, -10), 1.7, ivory3); //create sphere

    Sphere sphere3(Vec3f(0, 2, -10), 1.3, ivory2); //create sphere

    spheres.push_back(sphere);
    spheres.push_back(sphere2);
    spheres.push_back(sphere3);

    std::vector<Light> lights;
    lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));

    render(spheres, squares, lights);

    return 0;
}
