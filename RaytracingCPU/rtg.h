#pragma once

#include <array>
#include <vector>

#include "rtm.h"

namespace rtg
{

template<class T>
struct camera
{
	rtm::vector<T> position, forward;
	T h_fov, v_fov;

	constexpr rtg::camera<T>(
		const rtm::vector<T> & position = { 0, 0, 0 },
		const rtm::vector<T> & forward = { 0, 0, 1 },
		T h_fov = 70,
		T v_fov = 40) : position(position), forward(forward), h_fov(h_fov), v_fov(v_fov) { }
};

template<class T>
constexpr std::array<rtm::vector<T>, 4> view(const camera<T> & c) {
	auto
		r0 = rtm::rotate<T>(c.forward, static_cast<T>(c.h_fov / 2), static_cast<T>(0)),
		r1 = rtm::rotate<T>(c.forward, static_cast<T>(0), static_cast<T>(-c.v_fov / 2)),
		r2 = rtm::rotate<T>(c.forward, static_cast<T>(-c.h_fov / 2), static_cast<T>(0)),
		r3 = rtm::rotate<T>(c.forward, static_cast<T>(0), static_cast<T>( c.v_fov / 2));

	return {	r0 + r1, r1 + r2,
				r2 + r3, r3 + r0 };
}

template<class T>
constexpr rtm::ray<T> screen_ray(const rtm::vector<T> & position, const std::array<rtm::vector<T>, 4> view, size_t width, size_t height, size_t x, size_t y)
{
	T tw = x * static_cast<T>(1) / width,
		th = y * static_cast<T>(1) / height;

	return { position, rtm::interpolate(rtm::interpolate(view[0], view[1], tw), rtm::interpolate(view[3], view[2], tw), th) };
}

template<class T>
struct sphere : rtm::sphere<T>
{
	rtm::vector<T> color;

	constexpr rtg::sphere<T>(rtm::vector<T> center = { 0, 0, 0 }, T radius = 1, rtm::vector<T> color = { 1, 1, 1 }) : rtm::sphere<T>(center, radius), color(color) { }
};

template<class T>
struct scene
{
	std::vector<rtg::sphere<T>>	spheres;
	rtm::vector<T> light;

	rtg::scene<T>(const std::vector<rtg::sphere<T>> & spheres = {}, const rtm::vector<T> & light = {}) : spheres(spheres), light(light) { }
};

template<class T>
constexpr bool intersect(const rtm::ray<T> & ray, T max_distance, const std::vector<rtg::sphere<T>> & spheres, const std::vector<size_t> & exception_indexes,
	rtm::vector<T> * pIntersection_position, T * pIntersection_distance, size_t * pIntersection_sphere_index)
{
	bool intersect_found = false;

	T sqr_min_intersection_distance = max_distance * max_distance;
	for (size_t i = 0; i < spheres.size(); i++)
	{
		if (std::find(exception_indexes.begin(), exception_indexes.end(), i) != exception_indexes.end())
		{
			continue;
		}

		rtm::vector<T> intersection; T sqr_intersection_distance = 0;
		if (rtm::intersect(spheres[i], ray, &intersection) && (sqr_intersection_distance = rtm::sqr_magnitude(ray.start - intersection)) < sqr_min_intersection_distance)
		{
			sqr_min_intersection_distance = sqr_intersection_distance;
			if (pIntersection_position != nullptr)
			{
				*pIntersection_position = intersection;
			}
			if (pIntersection_distance != nullptr)
			{
				*pIntersection_distance = gcem::sqrt(sqr_intersection_distance);
			}
			if (pIntersection_sphere_index != nullptr)
			{
				*pIntersection_sphere_index = i;
			}
			intersect_found = true;
		}
	}

	return intersect_found;
}

}