#include "math_until.h"

#include <math.h>

point_t Math_Linear_Interpolation(const point_t *p_point_current, const point_t *p_point_end, const float step) {
    point_t point_target;
    point_target.x = p_point_current->x + (p_point_end->x - p_point_current->x) * step;
    point_target.y = p_point_current->y + (p_point_end->y - p_point_current->y) * step;
    point_target.z = p_point_current->z + (p_point_end->z - p_point_current->z) * step;

    return point_target;
}

void Math_Low_Pass_Filter(point_t *p_current, const point_t *p_target) {
    // smoothing factor, for smoother motion at low speed (0.0 -> 1.0)
    const float smooth_factor = 0.4f; 
    
    // STEP LIMIT (MAX SPEED): 
    const float max_step = 9.0f; 

    // Calculate the distance vector from the current position to the target point
    const float dx = p_target->x - p_current->x;
    const float dy = p_target->y - p_current->y;
    const float dz = p_target->z - p_current->z;

    // Calculate the step size if only the Low-Pass Filter is used
    float step_x = dx * smooth_factor;
    float step_y = dy * smooth_factor;
    float step_z = dz * smooth_factor;

    // Measure the actual length of this step vector in 3D space
    float step_length = sqrtf(step_x*step_x + step_y*step_y + step_z*step_z);

    // ================== ABS BRAKING MECHANISM ==================
    // If the calculated step is too large, clamp it down to exactly max_step!
    if (step_length > max_step) {
        float scale = max_step / step_length;
        step_x *= scale;
        step_y *= scale;
        step_z *= scale;
    }

    // Update the robot coordinates safely (Use the -> operator because it has been changed to a pointer)
    p_current->x += step_x;
    p_current->y += step_y;
    p_current->z += step_z;
}

point_t Math_Get_Parabolic_Arc_Point(point_t *p_point_current, point_t *p_point_end,const float height,const float t) {
    // LERP interpolation + Parabola
    point_t point_target = Math_Linear_Interpolation(p_point_current, p_point_end, t); // Initial linear interpolation point
    point_target.z += 4.0f * height * t * (1.0f - t);
    
    return point_target;
}