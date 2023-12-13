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
#include "filters.hpp"
#include "rotate.hpp"
#include "rpm.hpp"
#include "gendata.hpp"

//Configuration for test data generation
//I found some balls in the woods. My calipers tell me so.
#define BALL_DIAM_MM 42.67f
#define BALL_CIRC_MM (float)(BALL_DIAM_MM * M_PI)
#define DIMPLE_DIAM_MM (3.5f)
#define DIMPLE_DIAM_RATIO_BALL_CIRC (DIMPLE_DIAM_MM/BALL_CIRC_MM)
#define DIMPLE_DIAM_RATIO_BALL_DIAM (DIMPLE_DIAM_MM/BALL_DIAM_MM)
#define N_DIMPLES 400 //Estimate for whole ball
#define N_FRAMES 10 //NOTE: if we process all relationships, it's O(N*(N-1)
#define VISIBLE_RADIUS_OD .8 //Drop stuff beyond this
#define VISIBLE_RADIUS_ID .2 //Drop stuff close to center than this(glare)
#define DROP_RATIO .2 //drop some points randomly

//Configuration for shot
#define RPM_YAW 444
#define RPM_PITCH 9876
#define RPM_ROLL 333

//Can use this to prevent inlining for profiling
#define local static
//#define local static __attribute__((noinline))

local uint64_t randns(uint64_t range)
{
	return (float)range * (float)rand() / (float)RAND_MAX;
}

local vec3 randVec(float mag)
{
	vec3 v;

	v.x = rand() - RAND_MAX / 2;
	v.y = rand() - RAND_MAX / 2;
	v.z = rand() - RAND_MAX / 2;

	float normalize = mag/sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);

	v.x *= normalize;
	v.y *= normalize;
	v.z *= normalize;

	return v;
}

dataset genData(unsigned n, float d_yaw, float d_pitch, float d_roll)
{
	dataset ret;

	//Generate dimples, no masking for visibility
	uint64_t ts = 0;
	framestamp frame0 = {ts};
	for( unsigned i = 0; i < N_DIMPLES; ++i )
		{
		frame0.points.push_back(randVec(1.0f));
		}

	for( unsigned i = 0; i < n; ++i )
		{
		float yaw = d_yaw * ts / 1e9;
		float pitch = d_pitch * ts / 1e9;
		float roll = d_roll * ts / 1e9;

		//Rotate the vectors forward in time according to current timestamp
		framestamp frame = {ts};
		frame.points = rotate(frame0.points, yaw, pitch, roll);
//		std::cerr<<frame.points[0].x<<frame.points[0].y<<frame.points[0].z<<std::endl;
		frame = filterPct(frame, DROP_RATIO);
//		frame = filterVisible(frame, VISIBLE_RADIUS_ID, VISIBLE_RADIUS_OD);

		ret.frames.push_back(frame);
		ts += 588 * 1000; //1700fps
		ts += randns(78 * 1000); //difference between 1700fps and 1500fps
		}

	return ret;
}
