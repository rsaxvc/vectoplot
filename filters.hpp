#pragma once
#include "framestamp.hpp"

framestamp filterVisible(framestamp input, float visibleRadiusId, float visibleRadiusOd);
framestamp filterPct(framestamp input, float dropRatio);
