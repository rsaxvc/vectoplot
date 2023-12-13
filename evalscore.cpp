/*
This is an algorithm to estimate the spin of a ball, based
on only a large number of points around the ball, as the ball
flies. It is assumed that image processing has already occured,
and that suitable features detected on the ball. This algorithm
will process the (features + timestamps) and produce the spin.
*/
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "vec3.hpp"
#include "dataset.hpp"
#include "framestamp.hpp"
#include "gendata.hpp"
#include "rotate.hpp"
#include "score.hpp"

//Configuration for test data generation
//I found some balls in the woods. My calipers tell me so.
#define BALL_DIAM_MM 42.67f
#define BALL_CIRC_MM (float)(BALL_DIAM_MM * M_PI)
#define DIMPLE_DIAM_MM (3.5f)
#define DIMPLE_DIAM_RATIO_BALL_CIRC (DIMPLE_DIAM_MM/BALL_CIRC_MM)
#define DIMPLE_DIAM_RATIO_BALL_DIAM (DIMPLE_DIAM_MM/BALL_DIAM_MM)
#define N_DIMPLES 400 //Estimate for whole ball
#define N_FRAMES 10 //NOTE: if we process all relationships, it's O(N*(N-1)

//Can use this to prevent inlining for profiling
#define local static
//#define local static __attribute__((noinline))

local float dot(vec3 a, vec3 b)
{
return a.x * b.x + a.y * b.y + a.z * b.z;
}

local bool compareByX(const vec3 &a, const vec3 &b)
{
    return a.x < b.x;
}

//given a sorted vector, find the first element
//whose x-value is >= x;
local unsigned findGteX(const vecs & v, const float x)
{
	auto end = v.size();
	for(unsigned i = 0; i < end; ++i)
	{
		auto mid = (i + end)/2;
		auto midx = v[mid].x;
		if(midx < x)
		{
			//midpoint value is less than search target x,
			//advance to the midpoint, skipping everything between i and mid.
			i = mid;
			continue;
		}
		else if(v[i].x > x)
		{
			//This is the normal return path
			return i;
		}
		else if(midx > x)
		{
			//midpoint value is greater than search target x
			//reduce search area accordingly
			end = mid + 1;
		}
	}
	return v.size();
}

local float evalScore(const framestamp & f1, const framestamp & f2)
{
	float accum = 0;

	static constexpr float DOT_LIMIT = cos(DIMPLE_DIAM_RATIO_BALL_CIRC / 2);
	//Pre-compute the inverse at compile-time so we can use multiplication instead;
	//very, very slightly less accurate, yet faster :)
	static constexpr float DOT_LIMIT_D = 1 - DOT_LIMIT;
	static constexpr float DOT_LIMIT_D_INV = 1/(DOT_LIMIT_D);

	for( unsigned k = 0; k < f2.points.size(); ++k)
	{
		//These limits allow us to scan a limited portion of
		//the vectors for one frame. If they are too wide,
		//the search takes a little longer. If they are too narrow,
		//we might miss things.
		//These should be calculable but I haven't done it.
		float min_x = f2.points[k].x - DIMPLE_DIAM_RATIO_BALL_DIAM * .05;
		float max_x = f2.points[k].x + DIMPLE_DIAM_RATIO_BALL_DIAM * .05;
		auto end = f1.points.size();

		//Binary search f1 from the left, then scan right and terminate early
		auto start = findGteX(f1.points, min_x);

		for( auto l = start; l < end && f1.points[l].x <= max_x; ++l)
		{
			float d = dot(f2.points[k], f1.points[l]);
			if( d > DOT_LIMIT)
			{
				//very little thought was put into this cost function. It needs the following properties:
				//Return a higher value the closer d is to 1.
				//And, it needs offset by DOT_LIMIT, since without doing so, everything would be in the short range [DOT_LIMIT, 1]
				accum += (d - DOT_LIMIT) * DOT_LIMIT_D_INV;
				break;
			}
		}
	}
	return accum;
}

float evalScore(const dataset & data, float d_yaw, float d_pitch, float d_roll, bool only_nearby = false)
{
	float accum = 0;
	dataset rotated;
	rotated.frames.reserve(data.frames.size());
	for( unsigned i = 0; i < data.frames.size(); ++i)
		{
		//rotate all vectors backward to t=0
		const auto & f1 = data.frames[i];

		const float f1_yaw = d_yaw * f1.ns / 1e9;
		const float f1_pitch = d_pitch * f1.ns / 1e9;
		const float f1_roll = d_roll * f1.ns / 1e9;
		framestamp f1_rotated = {f1.ns};
		f1_rotated.points = rotate(f1.points, -f1_yaw, -f1_pitch, -f1_roll);

		//sort the vectors by X.
		std::sort(f1_rotated.points.begin(), f1_rotated.points.end(), compareByX);
		rotated.frames.push_back(f1_rotated);
		}

	for( unsigned i = 0; i < data.frames.size() - 1; ++i)
		{
		const auto & f1 = rotated.frames[i];

		for( unsigned j = i + 1; j < data.frames.size(); ++j)
			{
			if(!only_nearby || i + 1 == j)
				{
				const auto & f2 = rotated.frames[j];
				accum += evalScore(f1, f2);
				}
			}
		}
	return accum;
}

void findBestCoarse
	(
	const dataset & data,
	float yaw,
	float yaw_range,
	float yaw_grain,
	float pitch,
	float pitch_range,
	float pitch_grain,
	float roll,
	float roll_range,
	float roll_grain,
	float & best_score,
	float & best_d_yaw,
	float & best_d_pitch,
	float & best_d_roll,
	std::vector<score_t> & results
	)
{
	for( float d_yaw = yaw - yaw_range; d_yaw < yaw + yaw_range; d_yaw += yaw_grain)
	{
		for( float d_pitch = pitch - pitch_range; d_pitch < pitch + pitch_range; d_pitch += pitch_grain)
		{
			for( float d_roll = roll - roll_range; d_roll < roll + roll_range; d_roll += roll_grain)
			{
				float score = evalScore(data, d_yaw, d_pitch, d_roll, true);
				std::cout << d_yaw <<','<< d_pitch << ',' << d_roll << ',' << score << std::endl;

				score_t result = {score, {d_yaw, d_pitch, d_roll}};
				results.push_back(result);

				if( score > best_score)
				{
					best_score = score;
					best_d_yaw = d_yaw;
					best_d_pitch = d_pitch;
					best_d_roll = d_roll;
				}
			}
		}
	}
}
