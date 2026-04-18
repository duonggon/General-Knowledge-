#pragma once

#include "type_data.h"


// Square function
static inline float sqr(float x) {
    return x * x; // x^2
}

point_t Math_Linear_Interpolation(const point_t *p_point_current, const point_t *p_point_end, const float step);

void Math_Low_Pass_Filter(point_t *p_current, const point_t *p_target);

point_t Math_Get_Parabolic_Arc_Point(point_t *point_current, point_t *point_end,const float height,const float t);

