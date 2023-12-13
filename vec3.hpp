#pragma once
#include <vector>

struct vec3
	{
	float x;//yaw, d_yaw
	float y;//pitch, d_pitch
	float z;//roll, d_roll
	vec3 operator+(const vec3 & rhs)
		{
		vec3 r;
		r.x = x + rhs.x;
		r.y = y + rhs.y;
		r.z = z + rhs.z;
		return r;
		}
	vec3 & operator+=(const vec3 & rhs)
		{
		this->x += rhs.x;
		this->y += rhs.y;
		this->z += rhs.z;
		return *this;
		}
	vec3 & operator-(const vec3 & rhs)
		{
		this->x += rhs.x;
		this->y += rhs.y;
		this->z += rhs.z;
		return *this;
		}
	vec3 operator*(float rhs)
		{
		this->x *= rhs;
		this->y *= rhs;
		this->z *= rhs;
		return *this;
		}
	vec3 & operator/(unsigned rhs)
		{
		this->x /= rhs;
		this->y /= rhs;
		this->z /= rhs;
		return *this;
		}
	};

static inline vec3 operator*(float lhs, const vec3 & rhs)
	{
	vec3 r;
	r.x = lhs * rhs.x;
	r.y = lhs * rhs.y;
	r.z = lhs * rhs.z;
	return r;
	}

typedef std::vector<vec3> vecs;
