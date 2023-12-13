#pragma once
#include <vector>

#include "vec3.hpp"
#include "dataset.hpp"
#include "score.hpp"

float evalScore(const dataset & data, float d_yaw, float d_pitch, float d_roll, bool only_nearby = false);

static inline float evalScore(const dataset & data, const vec3 & v, bool only_nearby = false)
{
return evalScore(data, v.x, v.y, v.z, only_nearby);
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
    );
