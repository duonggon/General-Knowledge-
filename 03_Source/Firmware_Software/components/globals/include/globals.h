#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdint.h>

// MODE
#define MODE_HOMING 0
#define MODE_AUTOMATIC 1
#define MODE_MANUAL 2
#define MODE_PICK_AND_PLACE 3

typedef struct robot_delta robot_object_t;

// Assign GPIO pins to the arms
#define ARM_1 26
#define ARM_2 25
#define ARM_3 33 

// Declare the global variable that stores the robot state
extern robot_object_t *g_p_robot; 

// Queue 1: Transfer raw coordinates from UDP to the planning task (Planner) for smoothing
extern QueueHandle_t g_queue_udp_to_planner;  

// Queue 2: Transfer smoothed coordinates to the kinematics calculation task
extern QueueHandle_t g_queue_planner_to_kinematics;

// Queue 3: Transfer theta angles calculated by the IK algorithm to the motor control task
extern QueueHandle_t g_queue_kinematics_to_control;

// 
extern TaskHandle_t g_handle_planner;
extern TaskHandle_t g_handle_kinematics;