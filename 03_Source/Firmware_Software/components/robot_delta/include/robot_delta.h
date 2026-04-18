#pragma once

#include "type_data.h"
#include "task_kinematics.h"
#include "task_motor_control.h"
#include "task_planner.h"
#include "arm.h"

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"  // Library containing the SemaphoreHandle_t definition

typedef struct robot_delta{
    // robot parameters
    float A, RF, RE; // mm
    // effective workspace
    float Z_MIN, Z_MAX, R2; // R2 is the squared working radius

    // Variables that need to be locked when reading/writing
    // =============================================================
    // current robot state (the motors are controlled using these theta angles)
    point_t end_effector_current; // mm
    theta_t theta_current; // deg

    bool has_end_effector_current_changed; // end-effector 
    bool has_theta_current_changed; // theta

    // target robot state (stores the target point and target theta angles, kinematics are calculated using these variables)
    point_t end_effector_target; // mm
    theta_t theta_target; // deg

    bool has_end_effector_target_changed; // end-effector 
    bool has_theta_target_changed; // theta

    // homing interrupt flag
    bool should_break_homing;
    // ==============================================================
    SemaphoreHandle_t lock; // mutex to protect access to the robot data

    // Data structure for the robot's 3 arms
    arm_object_t _arm_1;
    arm_object_t _arm_2;
    arm_object_t _arm_3;

} robot_object_t;

robot_object_t Robot_Create(const float A, const float RF, const float RE, const float Z_MIN, const float Z_MAX, const float R2);

// This function runs when the ESP32 just starts up to set the initial point/angle
void Robot_Setup_Home_Point(robot_object_t *p_robot, theta_t *p_theta_home);