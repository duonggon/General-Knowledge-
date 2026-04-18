#pragma once

#include "globals.h"
#include "type_data.h"
#include "robot_delta.h"



// Call inverse kinematics to return the target angles
theta_t Kinematics_Call_Inverse(robot_object_t* p_robot, point_t *p_point_target);


// Call forward kinematics
point_t Kinematics_Call_Forward(robot_object_t* p_robot, theta_t *p_theta_target);