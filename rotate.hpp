#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include "vec3.hpp"

static std::vector<vec3> rotate(std::vector<vec3> input, float yaw, float pitch, float roll)
{
	const float ca = cosf(yaw);
	const float cb = cosf(pitch);
	const float cy = cosf(roll);
	const float sa = sinf(yaw);
	const float sb = sinf(pitch);
	const float sy = sinf(roll);

	const float m00 = ca * cb;
	const float m01 = ca * sb * sy - sa * cy;
	const float m02 = ca * sb * cy + sa * sy;
	const float m10 = sa * cb;
	const float m11 = sa * sb * sy + ca * cy;
	const float m12 = sa * sb * cy - ca * sy;
	const float m20 = -sb;
	const float m21 = cb * sy;
	const float m22 = cb * cy;

	for( unsigned i = 0; i < input.size(); ++i)
	{
		const float x = input[i].x;
		const float y = input[i].y;
		const float z = input[i].z;

		input[i].x = m00 * x + m01 * y + m02 * z;
		input[i].y = m10 * x + m11 * y + m12 * z;
		input[i].z = m20 * x + m21 * y + m22 * z;
	}

	return input;
}
