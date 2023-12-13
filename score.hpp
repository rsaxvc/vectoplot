#pragma once
#include "vec3.hpp"

struct score_t
    {
    float score;
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

