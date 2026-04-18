#include "kinematics.h"
#include "math_until.h"

#include <math.h>


static const int8_t _PHI[3] = {0, 120, -120}; // deg

// Rotate the coordinate system by an angle phi (0, 120, -120) around the Z axis
// Used to reduce the 3-arm problem to a 1-arm local problem
static inline point_t _Passive_Rotation(const point_t *p_point, int8_t phi)
{
    // phi: 0, 120, -120
    if (phi == 0)
    {
        return *p_point;
    }
 
    static const float _c = -0.5f;                       // cos(+-120)
    static const float _s_abs = 0.86602540378f;          // sin(120) = sqrt(3)/2

    point_t p_point_new;

    const float s = (phi > 0) ? _s_abs : -_s_abs;

    p_point_new.x = p_point->x * _c + p_point->y * s;
    p_point_new.y = -p_point->x * s + p_point->y * _c;
    p_point_new.z = p_point->z;

    return p_point_new;
}

// This function checks whether the target point lies within the robot's effective workspace.
static bool _Robot_Is_In_Workspace(const robot_object_t* p_robot, point_t *p_point){
    // check z height
    if (p_point->z > p_robot->Z_MAX || p_point->z < p_robot->Z_MIN)
        return false;

    // check whether the radius is smaller than R
    if (sqr(p_point->x) + sqr(p_point->y) > sqr(p_robot->R2))
        return false;

    return true;
}

static bool _Calculate_Kinematics_Inverse(const robot_object_t *self, point_t *p_point_target, theta_t *p_theta_target){
    if (!self || !p_point_target || !p_theta_target) return false;
    
    const float RF2 = sqr(self->RF);
    const float RE2 = sqr(self->RE);

    float *theta_future[3] = {&(p_theta_target->arm_1), &(p_theta_target->arm_2), &(p_theta_target->arm_3)};

    for (int8_t i = 0; i < 3; i++)
    {
        point_t E_i_arm_i = _Passive_Rotation(p_point_target, _PHI[i]); // Rotate the target end-effector point into the local coordinate system of arm i

        E_i_arm_i.x += self->A; // Ei coordinates for each arm

        const float d = sqrtf(sqr(E_i_arm_i.x - self->A) + sqr(E_i_arm_i.z));
        // Check for valid distance (avoid division by 0)
        if (d < 1e-6f)
            return false;

        // Calculate the intersection between the sphere (lower arm re) and the circle (upper arm rf)
        const float p = (RF2 - RE2 + sqr(E_i_arm_i.y) + sqr(d)) / (2 * d);
        
        // check
        if ((p > self->RF) || (p < 0))
            return false;

        const float H = sqrtf(RF2 - sqr(p));

        const point_t P_i = {self->A + p * (E_i_arm_i.x - self->A) / d,
                                0.0f,
                                p * (E_i_arm_i.z) / d,
                                0};

        // Perpendicular vector used to find the 2 intersection solutions
        const float vector[2] = {-E_i_arm_i.z, E_i_arm_i.x - self->A};

        point_t j[2];

        j[0].x = P_i.x + (H / d) * vector[0]; // x1
        j[0].z = P_i.z + (H / d) * vector[1]; // z1

        j[1].x = P_i.x - (H / d) * vector[0]; // x2
        j[1].z = P_i.z - (H / d) * vector[1]; // z2

        // choose the solution that satisfies the condition
        if ((j[0].x >= self->A) && (j[0].z <= 0))
            *theta_future[i] = atan2f(j[0].z, j[0].x - self->A);

        else if ((j[1].x >= self->A) && (j[1].z <= 0))
            *theta_future[i] = atan2f(j[1].z, j[1].x - self->A);

        else
            return false;
    }
    return true;
}


static bool _Calculate_Kinematics_Forward(const robot_object_t *self, theta_t *p_theta_target, point_t *p_point_target){
    // Check for null pointers to protect memory
    if (!self || !p_theta_target || !p_point_target) return false;
    
    // Extract variables from the struct into arrays to take advantage of loops
    const float theta_array[3] = {
        p_theta_target->arm_1,
        p_theta_target->arm_2,
        p_theta_target->arm_3
    };
    
    // variables storing joint coordinates in local and global coordinate systems
    point_t j_local[3];
    point_t j_global[3];

    // Calculate the joint positions in each arm's local coordinate system
    for (int8_t i = 0; i < 3; i++) {
        float sin_val, cos_val; 
        
        // Use sincosf to optimize performance, or replace with sinf()/cosf() if the compiler does not support it
        sincosf(theta_array[i], &sin_val, &cos_val);

        j_local[i].x = self->RF * cos_val;
        j_local[i].y = 0.0f;
        j_local[i].z = self->RF * sin_val;

        // Rotate each arm's coordinate system back to the global system
        // Pass _PHI[i] (assuming _PHI is a global array containing the 3 axis angles)
        j_global[i] = _Passive_Rotation(&j_local[i], -_PHI[i]); 
    }

    const float A1 = 2 * (j_global[0].x - j_global[1].x); 
    const float B1 = 2 * (j_global[1].y);                  
    const float C1 = 2 * (j_global[0].z - j_global[1].z); 
    const float D1 = sqr(j_global[0].x) - sqr(j_global[1].x) - sqr(j_global[1].y) + sqr(j_global[0].z) - sqr(j_global[1].z);

    const float A2 = 2 * (j_global[0].x - j_global[2].x); 
    const float B2 = 2 * (j_global[2].y);                  
    const float C2 = 2 * (j_global[0].z - j_global[2].z); 
    const float D2 = sqr(j_global[0].x) - sqr(j_global[2].x) - sqr(j_global[2].y) + sqr(j_global[0].z) - sqr(j_global[2].z);

    const float denom = A1 * B2 - A2 * B1;
    // Check denominator (avoid division by 0)
    if (fabsf(denom) < 1e-6f)
        return false;

    const float A = (-D1 * B2 + D2 * B1) / denom;
    const float C = (A1 * D2 - A2 * D1) / denom;
    const float B = (-C1 * B2 + C2 * B1) / denom;
    const float D = (A1 * C2 - A2 * C1) / denom;

    const float A_ = A - j_global[0].x;

    const float b = sqr(B) + sqr(D) + 1;
    const float c = 2 * B * A_ + 2 * C * D - 2 * j_global[0].z;
    const float d = sqr(A_) + sqr(C) + sqr(j_global[0].z) - sqr(self->RE);

    const float delta = sqr(c) - 4 * b * d;
    if (delta < 0)
        return false;

    // Choose the lower z solution (Delta robot points downward)
    const float z0 = fminf((-c + sqrtf(delta)) / (2 * b), (-c - sqrtf(delta)) / (2 * b));

    p_point_target->x = A + B * z0;
    p_point_target->y = C + D * z0;
    p_point_target->z = z0;

    return true;
}



// This function calls kinematics_inverse to calculate the theta angles from the target point, and updates the robot state.
theta_t Kinematics_Call_Inverse(robot_object_t* p_robot, point_t *p_point_target){
    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    bool has_end_effector_target_changed = p_robot->has_end_effector_target_changed; // Read the end_effector_target_changed flag into a local variable
    theta_t theta_target = p_robot->theta_current; // stores the calculated target theta angles from the target point
    xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
    

    if(has_end_effector_target_changed) { 
        // If the target point is not within the effective workspace, it will set the _has_end_effector_target_changed flag to false to indicate that the point is invalid.
        if (_Robot_Is_In_Workspace(p_robot, p_point_target) &&
            _Calculate_Kinematics_Inverse(p_robot, p_point_target, &theta_target))
        {   
            xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
            p_robot->theta_target = theta_target; // Update the target theta angles in the robot struct so other parts of the system can access and use them.
            p_robot->has_theta_target_changed = true; // Set this flag to true to indicate that the target theta angles have changed and need to be updated in the control system.
            xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
        } 
        xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
        p_robot->has_end_effector_target_changed = false; // Set this flag to false after using the target coordinates, to avoid recalculating if the target point has not changed.
        xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
    } else {
        xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
        theta_target = p_robot->theta_target; // Read the target theta angles into a local variable to return; if IK calculation fails, the old angles will be returned.
        xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
    }
    
    // Return the robot's target angles so that if FK cannot be calculated, the old angles can still be used
    return theta_target;
}



// This function calls kinematics_forward to calculate the end-effector position from the target theta angles
point_t Kinematics_Call_Forward(robot_object_t* p_robot, theta_t *p_theta_target){
    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    bool has_theta_target_changed = p_robot->has_theta_target_changed; // Read the theta_target_changed flag into a local variable
    xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
    
    point_t point_target;

    if(has_theta_target_changed) {
        if(_Calculate_Kinematics_Forward(p_robot, p_theta_target, &point_target)){
            xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
            p_robot->end_effector_target = point_target; // Update the end-effector position in the robot struct so other parts of the system can access and use it.
            p_robot->has_end_effector_target_changed = true; // Set this flag to true to indicate that the target end-effector position has changed and needs to be updated in the control system.
            xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
        }
        
        xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
        p_robot->has_theta_target_changed = false; // Set this flag to false after using the target angle set, to avoid recalculating if the theta angles have not changed.
        xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
    }

    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    point_target = p_robot->end_effector_target; // Read the target end-effector position into a local variable to return; if FK calculation fails, the old position will be returned.
    xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
    
    // Return the robot's target end-effector position so that if FK cannot be calculated, the old position can still be used
    return point_target;
}