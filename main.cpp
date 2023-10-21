#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#define BALL_DIAM_MM 42.67f
#define BALL_CIRC_MM (float)(BALL_DIAM_MM * M_PI)
#define DIMPLE_DIAM_MM (3.5f)
#define DIMPLE_DIAM_RATIO_BALL_CIRC (DIMPLE_DIAM_MM/BALL_CIRC_MM)
#define DIMPLE_DIAM_RATIO_BALL_DIAM (DIMPLE_DIAM_MM/BALL_DIAM_MM)
#define N_DIMPLES 400
#define N_FRAMES 20
#define RPM_YAW 0
#define RPM_PITCH 1000
#define RPM_ROLL 0
#define VISIBLE_RADIUS_OD .8
#define VISIBLE_RADIUS_ID .2
#define DROP_RATIO .2

#define local static
//#define local static __attribute__((noinline))

struct vec3
	{
	float x;
	float y;
	float z;
	};

typedef std::vector<vec3> vecs;

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
	const float ca = cos(yaw);
	const float cb = cos(pitch);
	const float cy = cos(roll);
	const float sa = sin(yaw);
	const float sb = sin(pitch);
	const float sy = sin(roll);

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

	float normalize = mag/fsqrt(v.x * v.x + v.y * v.y + v.z * v.z);

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
		frame = filterPct(frame, DROP_RATIO);
		frame = filterVisible(frame, VISIBLE_RADIUS_ID, VISIBLE_RADIUS_OD);

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

local unsigned findGteX(const vecs & v, const float x)
{
	auto end = v.size();
	for(unsigned i = 0; i < end; ++i)
	{
		if(v[i].x > x) return i;
		auto mid = (i + v.size())/2;
		auto midx = v[mid].x;
		if(x > midx)  i = mid;
		if(x < midx)  end = mid;
	}
	return v.size();
}

local float evalScore(const framestamp & f1, const framestamp & f2)
{
	float accum = 0;

	static constexpr float DOT_LIMIT = cos(DIMPLE_DIAM_RATIO_BALL_CIRC / 2);
	for( unsigned k = 0; k < f1.points.size(); ++k)
	{
		float min_x = f1.points[k].x - DIMPLE_DIAM_RATIO_BALL_DIAM * .75;
		auto start = findGteX(f1.points, min_x);

		for( unsigned l = start; l < f2.points.size(); ++l)
		{
			float d = dot(f1.points[k], f2.points[l]);
			if( d > DOT_LIMIT)
			{
				accum += (d - DOT_LIMIT) / (1 - DOT_LIMIT);
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

local void findBest
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
	float & best_rpm_yaw,
	float & best_rpm_pitch,
	float & best_rpm_roll
	)
{
	for( float rpm_yaw = yaw - yaw_range; rpm_yaw < yaw + yaw_range; rpm_yaw += yaw_grain)
	{
		float d_yaw = rpm_yaw / 60;
		for( float rpm_pitch = pitch - pitch_range; rpm_pitch < pitch + pitch_range; rpm_pitch += pitch_grain)
		{
			float d_pitch = rpm_pitch / 60;
			for( float rpm_roll = roll - roll_range; rpm_roll < roll + roll_range; rpm_roll += roll_grain)
			{
				float d_roll = rpm_roll / 60;
				uint64_t score = evalScore(data, d_yaw, d_pitch, d_roll);
				std::cout << d_yaw <<','<< d_pitch << ',' << d_roll << ',' << score << std::endl;
				if( score > best_score)
				{
					best_score = score;
					best_rpm_yaw = rpm_yaw;
					best_rpm_pitch = rpm_pitch;
					best_rpm_roll = rpm_roll;
				}
			}
		}
	}
}

int main()
{
float d_yaw = RPM_YAW * 2 * M_PI / 60;
float d_pitch = RPM_PITCH * 2 * M_PI / 60;
float d_roll = RPM_ROLL * 2 * M_PI / 60;

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
float best_rpm_yaw = 0, best_rpm_pitch = 2000, best_rpm_roll = 0;

float yaw_range = YAW_SEARCH_RPM;
float pitch_range = PITCH_SEARCH_RPM;
float roll_range = ROLL_SEARCH_RPM;
#define YAW_CHUNK 8
#define PITCH_CHUNK 15
#define ROLL_CHUNK 6
float yaw_step = 2 * yaw_range / YAW_CHUNK;
float pitch_step = 2 * pitch_range / PITCH_CHUNK;
float roll_step = 2 * roll_range / ROLL_CHUNK;

static const char * const stage_names[]={"coarse","mid","fine","ultra","hyper", "!!!"};

for( unsigned refine = 0; refine < 5; refine++)
	{
	findBest
		(
		data,
		best_rpm_yaw, yaw_range, yaw_step,
		best_rpm_pitch, pitch_range, pitch_step,
		best_rpm_roll, roll_range, roll_step,
		best_score, best_rpm_yaw, best_rpm_pitch, best_rpm_roll
		);
	yaw_range = yaw_step * 2;
	pitch_range = pitch_step * 2;
	roll_range = roll_step * 2;
	yaw_step = 2 * yaw_range / YAW_CHUNK;
	pitch_step = 2 * pitch_range / PITCH_CHUNK;
	roll_step = 2 * roll_range / ROLL_CHUNK;
	std::cerr<<stage_names[refine]<<"fix:"<<best_rpm_yaw<<','<<best_rpm_pitch<<','<<best_rpm_roll<<':'<<best_score<<std::endl;
	}

/*
for( int i = 900; i <= 1100; i += 1)
//for( int i = -5000; i <= 5000; i += 5)
	{
	float dp = d_pitch * (float)i / (float)1000;
	uint64_t score = evalScore(data, d_yaw, dp, d_roll);
	std::cout << dp <<','<< score << std::endl;
	}
*/

/*
volatile uint64_t temp = 0;
while(1)
	{
	temp += evalScore(data, d_yaw, d_pitch, d_roll);
	}
*/
}
