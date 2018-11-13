#include <fstream>
#include <iostream>
#include <execution>

#include "rtg.h"

constexpr rtm::vector<float> raytrace(const rtm::ray<float> & ray, const rtg::scene<float> & scene)
{
	rtm::vector<float> intersection_position;
	float intersection_distance = INFINITY;
	size_t intersection_sphere_index = -1;
	if (rtg::intersect(ray, INFINITY, scene.spheres, {}, &intersection_position, &intersection_distance, &intersection_sphere_index))
	{
		if (rtg::intersect({ intersection_position, scene.light - intersection_position }, rtm::magnitude(scene.light - intersection_position), scene.spheres,
			{ intersection_sphere_index }, static_cast<rtm::vector<float>*>(nullptr), static_cast<float*>(nullptr), nullptr))
		{
			return { 0.f, 0.f, 0.f };
		}

		const rtg::sphere<float> & intersection_sphere = scene.spheres[intersection_sphere_index];
		return { intersection_sphere.color * rtm::cos(intersection_sphere.center - intersection_position, intersection_position - scene.light) };
	}
	
	return { 0.f, 0.f, 0.f };
}

int main()
{
	std::ifstream input("input.txt");

	size_t width, height; input >> width >> height;
	rtg::camera<float> camera; 
	input >> camera.position.x >> camera.position.y >> camera.position.z;
	input >> camera.forward.x >> camera.forward.y >> camera.forward.z; camera.forward = rtm::normalize(camera.forward);
	input >> camera.h_fov >> camera.v_fov;
	camera.h_fov = rtm::deg2rad(camera.h_fov); camera.v_fov = rtm::deg2rad(camera.v_fov);
	auto view = rtg::view(camera);
	size_t spheres; input >> spheres;
	rtg::scene<float> scene;
	scene.spheres.resize(spheres);
	for (size_t i = 0; i < spheres; i++)
	{
		input >> scene.spheres[i].center.x >> scene.spheres[i].center.y >> scene.spheres[i].center.z;
		input >> scene.spheres[i].radius;
		input >> scene.spheres[i].color.x >> scene.spheres[i].color.y >> scene.spheres[i].color.z;
	}
	input >> scene.light.x >> scene.light.y >> scene.light.z;

	auto * image = static_cast<rtm::vector<float>*>(malloc(sizeof(rtm::vector<float>) * height * width));

	std::vector<size_t> pixel_indexes(height * width);
	for (size_t i = 0; i < pixel_indexes.size(); i++)
	{
		pixel_indexes[i] = i;
	}

	std::for_each(std::execution::par_unseq, pixel_indexes.begin(), pixel_indexes.end(), [&](const size_t & pixel) -> void {
		size_t i = pixel / width, j = pixel % width;
		image[i * width + j] = raytrace(rtg::screen_ray(camera.position, view, width, height, j, i), scene);
	});

	std::ofstream output("output.ppm", std::ios::binary);
	output << "P6 " << width << " " << height << " " << 255 << std::endl;
	for (size_t i = 0; i < height; i++)
	{
		for (size_t j = 0; j < width; j++)
		{
			output
				<< static_cast<char>(255 * rtm::clamp(0.f, 1.f, image[i * width + j].x))
				<< static_cast<char>(255 * rtm::clamp(0.f, 1.f, image[i * width + j].y))
				<< static_cast<char>(255 * rtm::clamp(0.f, 1.f, image[i * width + j].z));
		}
	}
	
	free(image);
}