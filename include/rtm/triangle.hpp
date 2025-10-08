#pragma once

#include "aabb.hpp"
#include <map>

namespace rtm
{

class alignas(64) Triangle
{
public:
	rtm::vec3 vrts[3];

public:
	Triangle() = default;

	Triangle(const rtm::vec3& v0, const rtm::vec3& v1, const rtm::vec3& v2)
	{
		this->vrts[0] = v0;
		this->vrts[1] = v1;
		this->vrts[2] = v2;
	}

	AABB aabb() const
	{
		AABB aabb;
		for(uint i = 0; i < 3; ++i)
			aabb.add(vrts[i]);
		return aabb;
	}

	static float cost() { return 1.0f; }

	rtm::vec3 normal()
	{
		return rtm::normalize(rtm::cross(vrts[0] - vrts[2], vrts[1] - vrts[2]));
	}

	float surface_area()
	{
		return rtm::length(rtm::cross(vrts[0] - vrts[2], vrts[1] - vrts[2])) / 2.0f;
	}
};

struct IntersectionTriangle
{
	Triangle tri;
	uint id;
};

inline uint decompress(const rtm::Triangle& in, uint id0, rtm::IntersectionTriangle* out)
{
	out[0].tri = in;
	out[0].id = id0;
	return 1;
}

}