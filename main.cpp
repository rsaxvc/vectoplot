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

//Configuration for test data generation
//I found some balls in the woods. My calipers tell me so.
#define BALL_DIAM_MM 42.67f
#define BALL_CIRC_MM (float)(BALL_DIAM_MM * M_PI)
#define DIMPLE_DIAM_MM (3.5f)
#define DIMPLE_DIAM_RATIO_BALL_CIRC (DIMPLE_DIAM_MM/BALL_CIRC_MM)
#define DIMPLE_DIAM_RATIO_BALL_DIAM (DIMPLE_DIAM_MM/BALL_DIAM_MM)
#define N_DIMPLES 400 //Estimate for whole ball
#define N_FRAMES 8 //NOTE: if we process all relationships, it's O(N*(N-1)
#define VISIBLE_RADIUS_OD .8 //Drop stuff beyond this
#define VISIBLE_RADIUS_ID .2 //Drop stuff close to center than this(glare)
#define DROP_RATIO .2 //drop some points randomly

//Configuration for shot
#define RPM_YAW 444
#define RPM_PITCH 9876
#define RPM_ROLL 333

//Configuration for solver - first takes priority
#define USE_HOT_GARBAGE 0 //do not use hot-garbage.
#define USE_NMS 0 //See: https://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method#One_possible_variation_of_the_NM_algorithm
#define USE_RWALK 1
#define USE_PROFILER 0 //infinite loop

//macro functions - presentation is in RPM, but processing is rad*hz
#define RPM2RADHZ(r) ((r) * 2.0 * M_PI / 60.0)
#define RADHZ2RPM(r) ((r) * 60.0 / (2.0 * M_PI))

//Can use this to prevent inlining for profiling
#define local static
//#define local static __attribute__((noinline))

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

vec3 operator*(float lhs, const vec3 & rhs)
	{
	vec3 r;
	r.x = lhs * rhs.x;
	r.y = lhs * rhs.y;
	r.z = lhs * rhs.z;
	return r;
	}

typedef std::vector<vec3> vecs;
struct score_t
	{
	uint64_t score;
	vec3 v;
	bool operator<(const score_t &b)
		{
		if(score != b.score) return score < b.score;
		return v.x < b.v.x;
		}
	bool operator>(const score_t &b)
		{
		if(score != b.score)return score > b.score;
		return v.x > b.v.x;
		}
	};

struct framestamp
	{
	uint64_t ns;
	vecs points;
	};

struct dataset
	{
	std::vector<framestamp> frames;
	};

local std::vector<vec3> rotate(std::vector<vec3> input, float yaw, float pitch, float roll)
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

local framestamp filterVisible(framestamp input, float visibleRadiusId, float visibleRadiusOd)
{
	const float vrid2 = visibleRadiusId * visibleRadiusId;
	const float vrod2 = visibleRadiusOd * visibleRadiusOd;
	framestamp f = {input.ns};

	for( unsigned i = 0; i < input.points.size(); ++i)
		{
		const vec3 p = input.points[i];
		const float r = p.x * p.x + p.y * p.y;
		//point must not be inside ID, must be inside OD, and must be Z+
		if(r > vrid2 && r < vrod2 && p.z > 0)
			{
			f.points.push_back(p);
			}
		}
	return f;
}

local framestamp filterPct(framestamp input, float dropRatio)
{
	const int dropMax = RAND_MAX * dropRatio;
	framestamp f = {input.ns};
	for( unsigned i = 0; i < input.points.size(); ++i)
		{
		const vec3 p = input.points[i];
		if(rand() > dropMax)
			{
			f.points.push_back(p);
			}
		}
	return f;
}

local dataset genData(unsigned n, float d_yaw, float d_pitch, float d_roll)
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

local uint64_t evalScore(const dataset & data, float d_yaw, float d_pitch, float d_roll)
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
			const auto & f2 = rotated.frames[j];
			accum += evalScore(f1, f2);
			}
		}
	return accum;
}

local uint64_t evalScore(const dataset & data, const vec3 & v)
{
return evalScore(data, v.x, v.y, v.z);
}

local void findBestCoarse
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
	uint64_t & best_score,
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
				uint64_t score = evalScore(data, d_yaw, d_pitch, d_roll);
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
uint64_t best_score = 0;
float best_d_yaw = 0, best_d_pitch = 0, best_d_roll = 0;

float yaw_range = RPM2RADHZ(YAW_SEARCH_RPM);
float pitch_range = RPM2RADHZ(PITCH_SEARCH_RPM);
float roll_range = RPM2RADHZ(ROLL_SEARCH_RPM);
#define YAW_CHUNK 12
#define PITCH_CHUNK 25
#define ROLL_CHUNK 10
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
for(unsigned i = 0; i < 10000; ++i)
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
	max_x += .1 * dx;
	max_y += .1 * dy;
	max_z += .1 * dz;
	min_x -= .1 * dx;
	min_y -= .1 * dy;
	min_z -= .1 * dz;

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
