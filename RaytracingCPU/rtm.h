#pragma once

#include "gcem.hpp"

namespace rtm
{

constexpr long double pi = 3.141592653589793115997963468544185161590576171875l;

template <class T>
constexpr T deg2rad(const T deg)
{
	return deg / static_cast<T>(180) * static_cast<T>(pi);
}

template <class T>
constexpr T rad2deg(const T rad)
{
	return rad / static_cast<T>(pi) * static_cast<T>(180);
}

template <class T>
struct vector
{
	T x, y, z;

	constexpr rtm::vector<T>(T x = 0, T y = 0, T z = 0) : x(x), y(y), z(z) { }
};

template <class T>
constexpr rtm::vector<T> operator+(const rtm::vector<T> & a, const rtm::vector<T> & b)
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

template <class T>
constexpr rtm::vector<T> operator-(const rtm::vector<T> & a, const rtm::vector<T> & b)
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

template <class T, class V>
constexpr rtm::vector<V> operator*(const rtm::vector<T> & a, V v)
{
	return { a.x * v, a.y * v, a.z * v };
}

template <class T, class V>
constexpr rtm::vector<V> & operator*=(rtm::vector<T> & a, V v)
{
	a.x *= v; a.y *= v; a.z *= v;
	return a;
}

template <class T>
std::ostream & operator<<(std::ostream & output, const rtm::vector<T> a)
{
	output << "{ " << a.x << ", " << a.y << ", " << a.z << " }";
	return output;
}

template <class T>
const std::istream & operator>>(const rtm::vector<T> a, const std::istream & input)
{
	input >> a.x >> a.y >> a.z;
	return input;
}

template <class T>
constexpr T sqr_magnitude(const rtm::vector<T> & a)
{
	return a.x * a.x + a.y * a.y + a.z * a.z;
}

template <class T>
constexpr T magnitude(const rtm::vector<T> & a)
{
	return gcem::sqrt(rtm::sqr_magnitude(a));
}

template <class T>
constexpr T dot(const rtm::vector<T> & a, const rtm::vector<T> & b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <class T>
constexpr T cos(const rtm::vector<T> & a, const rtm::vector<T> & b)
{
	return rtm::dot(a, b) / (rtm::magnitude(a) * rtm::magnitude(b));
}

template <class T>
constexpr rtm::vector<T> normalize(const rtm::vector<T> & a)
{
	T magnitude = rtm::magnitude(a);
	return { a.x / magnitude, a.y / magnitude, a.z / magnitude };
}

template <class T>
constexpr rtm::vector<T> projxy(const rtm::vector<T> & a)
{
	return { a.x, a.y, static_cast<T>(0) };
}

template <class T>
constexpr rtm::vector<T> projxz(const rtm::vector<T> & a)
{
	return { a.x, static_cast<T>(0), a.z };
}

template <class T>
constexpr rtm::vector<T> projyz(const rtm::vector<T> & a)
{
	return { static_cast<T>(0), a.y, a.z };
}

template <class T>
constexpr rtm::vector<T> rotate(const rtm::vector<T> & a, T phi, T theta)
{
	T phi0 = 0, theta0 = gcem::acos(rtm::cos(a, { 0, 0, 1 })), r = rtm::magnitude(a);

	rtm::vector a_projxy = rtm::projxy(a);
	if (rtm::sqr_magnitude(a) == static_cast<T>(0))
	{
		phi0 = static_cast<T>(0);
	}
	else
	{
		phi0 = gcem::acos(rtm::cos(a_projxy, { static_cast<T>(1), static_cast<T>(0), static_cast<T>(0) }));
	}

	theta0 = gcem::acos(rtm::cos(a, { static_cast<T>(0), static_cast<T>(0), static_cast<T>(1) }));

	phi += phi0;
	theta += theta0;

	return { r * gcem::sin(theta) * gcem::cos(phi), r * gcem::sin(theta) * gcem::sin(phi), r * gcem::cos(theta) };
}

template <class T>
constexpr rtm::vector<T> interpolate(const rtm::vector<T> & a, const rtm::vector<T> & b, T t)
{
	return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
}

template <class T>
constexpr rtm::vector<T> mix(const rtm::vector<T> & a, const rtm::vector<T> & b, T k)
{
	return a * k + b * (1 - k);
}

template <class T>
struct ray
{
	rtm::vector<T> start, direction;

	constexpr rtm::ray<T>(const rtm::vector<T> & start = { 0, 0, 0 }, const rtm::vector<T> & dir = { 0, 0, 0 }) : start(start), direction(rtm::normalize(dir)) { }
};

template <class T>
struct sphere
{
	rtm::vector<T> center;
	T radius;

	constexpr sphere(const rtm::vector<T> & center = { 0, 0, 0 }, T radius = 1) : center(center), radius(radius) { }
};

template <class T>
constexpr bool intersect(const rtm::sphere<T> & s, const rtm::ray<T> & r, rtm::vector<T> * pIntersection)
{
	rtm::vector<T> a = s.center - r.start;

	T D = rtm::sqr_magnitude(a) * (rtm::cos(r.direction, a) * rtm::cos(r.direction, a) - static_cast<T>(1)) + s.radius * s.radius;
	if (D < 0) { return false; }
	
	T m = (rtm::magnitude(a) * rtm::cos(r.direction, a) - gcem::sqrt(D));
	if (m < 0) { return false; }

	rtm::vector<T> b = rtm::normalize(r.direction) * m;

	if (pIntersection != nullptr)
	{
		*pIntersection = r.start + b;
	}

	return true;
}

template <class T>
constexpr T clamp(T min, T max, T value)
{
	return (value >= max ? max : value <= min ? min : value);
}

}