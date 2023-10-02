#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#define N_DIMPLES 400
#define N_FRAMES 20
#define RPM_YAW 0
#define RPM_PITCH 1000
#define RPM_ROLL 0
#define VISIBLE_RADIUS_OD .8
#define VISIBLE_RADIUS_ID .2

#define local static
//#define local static __attribute__((noinline))

struct vec3
	{
	float x;
	float y;
	float z;
	};

struct framestamp
	{
	uint64_t ns;
	std::vector<vec3> points;
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
		vec3 p = input.points[i];
		float r = p.x * p.x + p.y * p.y;
		//point must not be inside ID, must be inside OD, and must be Z+
		if(r > vrid2 && r < vrod2 && p.z > 0)
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
		rotated.frames.push_back(f1_rotated);
		}

	for( unsigned i = 0; i < data.frames.size() - 1; ++i)
		{
		const auto & f1 = rotated.frames[i];

		for( unsigned j = i + 1; j < data.frames.size(); ++j)
			{
			const auto & f2 = rotated.frames[j];

			static const float DOT_LIMIT = .999995;
//			static const float DOT_LIMIT = .9995;
			for( unsigned k = 0; k < f1.points.size(); ++k)
				{
				for( unsigned l = 0; l < f2.points.size(); ++l)
					{
					float d = dot(f1.points[k], f2.points[l]);
					if( d > DOT_LIMIT)
						{
						accum += (d - DOT_LIMIT) / (1 - DOT_LIMIT);
						break;
						}
					}
				}
			}
		}
	return accum;
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

std::cout << "d_pitch,score" << std::endl;
for( int i = 900; i <= 1100; i += 1)
//for( int i = -5000; i <= 5000; i += 25)
	{
	float dp = d_pitch * (float)i / (float)1000;
	uint64_t score = evalScore(data, d_yaw, dp, d_roll);
	std::cout << dp <<','<< score << std::endl;
	}

/*
volatile uint64_t temp = 0;
while(1)
	{
	temp += evalScore(data, d_yaw, d_pitch, d_roll);
	}
*/
}
