#pragma once

#include "type_data.h"
#include "robot_delta.h"
#include <stdbool.h> // Required to use bool as the return type

typedef struct {
    // ==========================================
    int current_step;   // Current step (starts from 0)
    int total_steps;    // Total number of steps to complete one trajectory
    int current_type;   // Type of trajectory currently running (1: Straight line, 2: Circle)

    // ==========================================
    float z;            // Fixed Z height (e.g. -290.0 mm)
    float R;            // Configuration radius for the Auto circle (e.g. 130.0 mm)
    int auto_state;     // Scenario management: 0 (Initialize), 1 (Priming Line), 2 (Looping Circle)

    // ==========================================
    // Used for the Start_Line function
    float start_x, start_y, start_z;
    float end_x, end_y, end_z;

    // Used for the Start_Circle function
    float center_x, center_y, center_z;
    float radius;       // Actual radius passed into the circle drawing function
} trajectory_t;

// Pass in the whole "brain" (traj) to store data, and the "muscle" (robot) to get the starting point
void Start_Line(trajectory_t *p_traj, robot_object_t *p_robot, float end_x, float end_y, float end_z, int total_steps);

// Pass in the "brain" (traj) and the circle parameters
void Start_Circle(trajectory_t *p_traj, float center_x, float center_y, float center_z, float radius, int total_steps);

// Function to output interpolated coordinates (Must have)
bool Get_Next_Point(trajectory_t *p_traj, point_t *p_point);