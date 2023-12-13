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
#include "evalscore.hpp"
#include "framestamp.hpp"
#include "gendata.hpp"
#include "rpm.hpp"
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

//Configuration for shot
#define RPM_YAW 444
#define RPM_PITCH 9876
#define RPM_ROLL 333

//Configuration for solver - first takes priority
#define USE_HOT_GARBAGE 0 //do not use hot-garbage.
#define USE_NMS 0 //See: https://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method#One_possible_variation_of_the_NM_algorithm
#define USE_RWALK 1
#define USE_PROFILER 0 //infinite loop

//Can use this to prevent inlining for profiling
#define local static
//#define local static __attribute__((noinline))

static uint64_t gettime_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

#if USE_NMS
local vec3 centroid(const std::vector<score_t> & input, size_t n)
{
vec3 r = {0,0,0};
for( size_t i = 0; i < n; ++i)
	{
	r += input[i].v;
	}
return r / n;
}
#endif

local float randf(float min, float max)
{
float r =(float)rand() / (float)RAND_MAX;
float d = max - min;
return min + d * r;
}

int main()
{
float d_yaw = RPM2RADHZ(RPM_YAW);
float d_pitch = RPM2RADHZ(RPM_PITCH);
float d_roll = RPM2RADHZ(RPM_ROLL);

//std::cout << "Sim Params:"<<d_yaw<<','<<d_pitch<<','<<d_roll << std::endl;

dataset data = genData(N_FRAMES, d_yaw, d_pitch, d_roll);
/*
for( unsigned i = 0; i < data.frames.size(); ++i)
	{
	std::cout << "ts=" << data.frames[i].ns / 1e6 << " nPts=" << data.frames[i].points.size() << std::endl;
	for( unsigned j = 0; j < data.frames[i].points.size() && j < 5; ++j)
		{
		std::cout
			<< '\t'
			<< '('
			<< data.frames[i].points[j].x
			<< ','
			<< data.frames[i].points[j].y
			<< ','
			<< data.frames[i].points[j].z
			<< ')'
			<< std::endl;
		}
	}
*/

std::cout << "d_yaw,d_pitch,d_roll,score" << std::endl;
#define YAW_SEARCH_RPM 2500
#define PITCH_SEARCH_RPM 12000
#define ROLL_SEARCH_RPM 500
float best_score = 0;
float best_d_yaw = 0, best_d_pitch = 0, best_d_roll = 0;

float yaw_range = RPM2RADHZ(YAW_SEARCH_RPM);
float pitch_range = RPM2RADHZ(PITCH_SEARCH_RPM);
float roll_range = RPM2RADHZ(ROLL_SEARCH_RPM);
#define YAW_CHUNK 10
#define PITCH_CHUNK 15
#define ROLL_CHUNK 5
float yaw_step = yaw_range / YAW_CHUNK;
float pitch_step = pitch_range / PITCH_CHUNK;
float roll_step = roll_range / ROLL_CHUNK;

std::vector<score_t> results;

#if USE_HOT_GARBAGE
//An example solver(hot-garbage approach)
static const char * const stage_names[]={"coarse","mid","fine","ultra","hyper", "!!!"};

for( unsigned refine = 0; refine < 5; refine++)
	{
	findBestCoarse
		(
		data,
		best_d_yaw, yaw_range, yaw_step,
		best_d_pitch, pitch_range, pitch_step,
		best_d_roll, roll_range, roll_step,
		best_score, best_d_yaw, best_d_pitch, best_d_roll,
		results
		);
	yaw_range = yaw_step * 2;
	pitch_range = pitch_step * 2;
	roll_range = roll_step * 2;
	yaw_step = 2 * yaw_range / YAW_CHUNK;
	pitch_step = 2 * pitch_range / PITCH_CHUNK;
	roll_step = 2 * roll_range / ROLL_CHUNK;
	std::cerr<<stage_names[refine]<<"fix:"
		<<RADHZ2RPM(best_d_yaw)<<','
		<<RADHZ2RPM(best_d_pitch)<<','
		<<RADHZ2RPM(best_d_roll)<<':'
		<<best_score<<std::endl;
	}
#elif USE_NMS
/*A hybrid brute-Nelder-Mead-simplex solver

Pass1: Because NMS gets stuck in local maxima easily, we brute-force scan for the best few points.
Pass2: Nelder-Mead Simplex climbs the gradient hill formed by alinging more and more dimples more closely.
Pass3: profit???
*/
findBestCoarse
	(
	data,
	best_d_yaw, yaw_range, yaw_step,
	best_d_pitch, pitch_range, pitch_step,
	best_d_roll, roll_range, roll_step,
	best_score, best_d_yaw, best_d_pitch, best_d_roll,
	results
	);
std::cerr<<"brute"<<"fix:"
	<<RADHZ2RPM(best_d_yaw)<<','
	<<RADHZ2RPM(best_d_pitch)<<','
	<<RADHZ2RPM(best_d_roll)<<':'
	<<best_score<<std::endl;

//See: https://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method#One_possible_variation_of_the_NM_algorithm
//Except: ours is inverted(hill climber rather than minimizer)
std::sort(results.begin(), results.end(), std::greater<>());
std::vector<score_t> nms_state;
//Grab top N
for(size_t i = 0; i < 15; ++i)
	{
	nms_state.push_back(results[i]);
	}

for(unsigned i = 0; i < nms_state.size(); ++i)
	{
	std::cout << nms_state[i].v.x <<',' << nms_state[i].v.y << ',' << nms_state[i].v.z << ',' << nms_state[i].score << std::endl;
	}

//Manual loop count for now.
for(unsigned i = 0; i < 30; ++i)
	{
	//Step 1: sort the list.
	std::sort(nms_state.begin(), nms_state.end(), std::greater<>());

	//Debug: print out top entries
	std::cerr << "NMS" << i << std::endl;
	for(unsigned i = 0; i < nms_state.size(); ++i)
		{
		std::cerr << '\t'
		<< "score: "<<nms_state[i].score << ' '
		<< RADHZ2RPM(nms_state[i].v.x) << ' '
		<< RADHZ2RPM(nms_state[i].v.y) << ' '
		<< RADHZ2RPM(nms_state[i].v.z) << std::endl;
		}

	//Step 2: centroid for everything but last
	vec3 x0 = centroid(nms_state, nms_state.size()-1);
	score_t xbest = nms_state.front();
	score_t xworst = nms_state.back();

	std::cerr << '\t'
		<< "x0: "
		<< RADHZ2RPM(x0.x) << ' '
		<< RADHZ2RPM(x0.y) << ' '
		<< RADHZ2RPM(x0.z) << std::endl;

	//Step 3: Reflection
	static const float alpha = .25f;
	score_t xr;
	xr.v = x0 + alpha * (x0 - xworst.v);
	xr.score = evalScore(data, xr.v);

	std::cerr << '\t'
		<< "xr:    "<<xr.score << ' '
		<< RADHZ2RPM(xr.v.x) << ' '
		<< RADHZ2RPM(xr.v.y) << ' '
		<< RADHZ2RPM(xr.v.z) << std::endl;

	std::cerr <<"nms xbest.score=" << xbest.score << " xr.score=" <<xr.score <<" xworst.score=" << xworst.score << std::endl;
	if(xr.score < xbest.score && xr.score > xworst.score)
		{
		//if we are better than worst
		std::cerr << "reflecting" << std::endl;
		nms_state.back() = xr;
		std::cout << xr.v.x <<','<< xr.v.y << ',' << xr.v.z << ',' << xr.score << std::endl;
		continue;
		}

	//Step 4: Expansion
	if(xr.score > xbest.score)
		{
		std::cerr << "expanding" << std::endl;
		static const float gamma = 1.25f;
		score_t xe;
		xe.v = x0 + gamma * (xr.v - x0);
		xe.score = evalScore(data, xe.v);
		nms_state.back() = xe.score > xr.score ? xe : xr;
		continue;
		}

	//Step 5: Contraction
	const float rho = .25;
	if(xr.score >= xworst.score)
		{
		score_t xc;
		xc.v = x0 + rho * (xr.v - x0);
		xc.score = evalScore(data, xc.v);
		if(xc.score > xr.score)
			{
			nms_state.back() = xc;
			std::cerr << "contracting" << std::endl;
			continue;
			}
		}
	else
		{
		score_t xc;
		xc.v = x0 + rho * (xworst.v - x0);
		xc.score = evalScore(data, xc.v);
		if(xc.score > xworst.score)
			{
			nms_state.back() = xc;
			std::cerr << "contracting" << std::endl;
			continue;
			}
		}

	//Step 6: Shrink
	std::cerr << "shrinking" << std::endl;
	for(size_t i = 1; i < nms_state.size(); ++i)
		{
		score_t xtemp;
		xtemp.v = xbest.v + .25f * (nms_state[i].v - xbest.v);
		xtemp.score = evalScore(data, xtemp.v);
		nms_state[i] = xtemp;
		}
	}

#elif USE_RWALK
/*A hybrid brute-walker
Pass3: profit???
*/
findBestCoarse
	(
	data,
	best_d_yaw, yaw_range, yaw_step,
	best_d_pitch, pitch_range, pitch_step,
	best_d_roll, roll_range, roll_step,
	best_score, best_d_yaw, best_d_pitch, best_d_roll,
	results
	);
std::cerr<<"brute"<<"fix:"
	<<RADHZ2RPM(best_d_yaw)<<','
	<<RADHZ2RPM(best_d_pitch)<<','
	<<RADHZ2RPM(best_d_roll)<<':'
	<<best_score<<std::endl;

//See: https://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method#One_possible_variation_of_the_NM_algorithm
//Except: ours is inverted(hill climber rather than minimizer)
std::sort(results.begin(), results.end(), std::greater<>());
std::vector<score_t> nms_state;
//Grab top N
for(size_t i = 0; i < 25; ++i)
	{
	nms_state.push_back(results[i]);
	}

//Manual loop count for now.
for(unsigned i = 0; i < 150; ++i)
	{
	auto & xworst = nms_state.back();
	float min_x = INFINITY,min_y = INFINITY,min_z=INFINITY,max_x=-INFINITY,max_y=-INFINITY,max_z=-INFINITY;
	for(auto i : nms_state)
		{
		if(i.v.x < min_x) min_x = i.v.x;
		if(i.v.x > max_x) max_x = i.v.x;
		if(i.v.y < min_y) min_y = i.v.y;
		if(i.v.y > max_y) max_y = i.v.y;
		if(i.v.z < min_z) min_z = i.v.z;
		if(i.v.z > max_z) max_z = i.v.z;
		}

	float dx = max_x - min_x;
	float dy = max_y - min_y;
	float dz = max_z - min_z;
	max_x += .3 * dx + 10;
	max_y += .3 * dy + 10;
	max_z += .3 * dz + 10;
	min_x -= .3 * dx - 10;
	min_y -= .3 * dy - 10;
	min_z -= .3 * dz - 10;

	for(unsigned j = 0; j < 10000; ++j)
		{
		score_t xr;
		xr.v.x = randf(min_x,max_x);
		xr.v.y = randf(min_y,max_y);
		xr.v.z = randf(min_z,max_z);
		xr.score = evalScore(data, xr.v);
		if(xr.score >= xworst.score)
			{
			std::cerr<<"derping:\t"<<i<<",\t"<<j<<",\t"<<xr.score<<std::endl;
			std::cout << xr.v.x <<','<< xr.v.y << ',' << xr.v.z << ',' << xr.score << std::endl;

			//Step 1: sort the list.
			xworst = xr;
			std::sort(nms_state.begin(), nms_state.end(), std::greater<>());
			break;
			}
		}

	}

#elif USE_PROFILER
//Infinite loop for profiling
volatile uint64_t temp = 0;

while(1)
	{
	static const unsigned COUNT = 1000;
	uint64_t start = gettime_ns();
	for(unsigned i = 0; i < COUNT; ++i)
		{
		temp += evalScore(data, d_yaw, d_pitch, d_roll);
		}
	uint64_t delta = gettime_ns() - start;
	std::cerr<<"did "<< 1.0e9*(double)COUNT/(double)delta << " evalScore()s per second"<<std::endl;
	}
#else
	#error "nothing to do"
#endif
}
