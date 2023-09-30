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
	double x;
	double y;
	double z;
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

local std::vector<vec3> rotate(std::vector<vec3> input, double yaw, double pitch, double roll)
{
	const double ca = cos(yaw);
	const double cb = cos(pitch);
	const double cy = cos(roll);
	const double sa = sin(yaw);
	const double sb = sin(pitch);
	const double sy = sin(roll);

	const double m00 = ca * cb;
	const double m01 = ca * sb * sy - sa * cy;
	const double m02 = ca * sb * cy + sa * sy;
	const double m10 = sa * cb;
	const double m11 = sa * sb * sy + ca * cy;
	const double m12 = sa * sb * cy - ca * sy;
	const double m20 = -sb;
	const double m21 = cb * sy;
	const double m22 = cb * cy;

	for( unsigned i = 0; i < input.size(); ++i)
	{
		const double x = input[i].x;
		const double y = input[i].y;
		const double z = input[i].z;

		input[i].x = m00 * x + m01 * y + m02 * z;
		input[i].y = m10 * x + m11 * y + m12 * z;
		input[i].z = m20 * x + m21 * y + m22 * z;
	}

	return input;
}

local uint64_t randns(uint64_t range)
{
	return (double)range * (double)rand() / (double)RAND_MAX;
}

local vec3 randVec(double mag)
{
	vec3 v;

	v.x = rand() - RAND_MAX / 2;
	v.y = rand() - RAND_MAX / 2;
	v.z = rand() - RAND_MAX / 2;

	double normalize = mag/fsqrt(v.x * v.x + v.y * v.y + v.z * v.z);

	v.x *= normalize;
	v.y *= normalize;
	v.z *= normalize;

	return v;
}

local framestamp filterVisible(framestamp input, double visibleRadiusId, double visibleRadiusOd)
{
	const double vrid2 = visibleRadiusId * visibleRadiusId;
	const double vrod2 = visibleRadiusOd * visibleRadiusOd;
	framestamp f = {input.ns};

	for( unsigned i = 0; i < input.points.size(); ++i)
		{
		vec3 p = input.points[i];
		double r = p.x * p.x + p.y * p.y;
		//point must not be inside ID, must be inside OD, and must be Z+
		if(r > vrid2 && r < vrod2 && p.z > 0)
			{
			f.points.push_back(p);
			}
		}
	return f;
}

local dataset genData(unsigned n, double d_yaw, double d_pitch, double d_roll)
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
		double yaw = d_yaw * ts / 1e9;
		double pitch = d_pitch * ts / 1e9;
		double roll = d_roll * ts / 1e9;

		framestamp frame = {ts};
		frame.points = rotate(frame0.points, yaw, pitch, roll);
		frame = filterVisible(frame, VISIBLE_RADIUS_ID, VISIBLE_RADIUS_OD);

		ret.frames.push_back(frame);
		ts += 588 * 1000; //1700fps
		ts += randns(78 * 1000); //difference between 1700fps and 1500fps
		}

	return ret;
}

local double dot(vec3 a, vec3 b)
{
return a.x * b.x + a.y * b.y + a.z * b.z;
}

local uint64_t evalScore(const dataset & data, double d_yaw, double d_pitch, double d_roll)
{
	double accum = 0;
	for( unsigned i = 0; i < data.frames.size() - 1; ++i)
		{
		const auto & f1 = data.frames[i];
		for( unsigned j = i + 1; j < data.frames.size(); ++j)
			{
			const auto & f2 = data.frames[j];
			const uint64_t ts = f2.ns - f1.ns;
			const double yaw = d_yaw * ts / 1e9;
			const double pitch = d_pitch * ts / 1e9;
			const double roll = d_roll * ts / 1e9;

			std::vector<vec3> f1_rotated = rotate(f1.points, yaw, pitch, roll);
			static const double DOT_LIMIT = .999995;
//			static const double DOT_LIMIT = .9995;
			for( unsigned k = 0; k < f1_rotated.size(); ++k)
				{
				for( unsigned l = 0; l < f2.points.size(); ++l)
					{
					double d = dot(f1_rotated[k], f2.points[l]);
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
double d_yaw = RPM_YAW * 2 * M_PI / 60;
double d_pitch = RPM_PITCH * 2 * M_PI / 60;
double d_roll = RPM_ROLL * 2 * M_PI / 60;

std::cout << "Sim Params:"<<d_yaw<<','<<d_pitch<<','<<d_roll << std::endl;

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
	double dp = d_pitch * (double)i / (double)1000;
	uint64_t score = evalScore(data, d_yaw, dp, d_roll);
	std::cout << dp <<','<< score << std::endl;
	}

volatile uint64_t temp = 0;
while(1)
	{
	temp += evalScore(data, d_yaw, d_pitch, d_roll);
	}
}
