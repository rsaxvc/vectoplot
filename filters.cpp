#include "filters.hpp"
#include "framestamp.hpp"
#include <cstdlib>

framestamp filterVisible(framestamp input, float visibleRadiusId, float visibleRadiusOd)
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

framestamp filterPct(framestamp input, float dropRatio)
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

