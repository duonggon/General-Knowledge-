#include "task_planner.h"

#include "type_data.h"
#include "globals.h"
#include "robot_delta.h"
#include "kinematics.h"
#include "math_until.h"
#include "trajectory.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h" // Logging library

// Define a name (TAG) for easier log filtering in the terminal
static const char *TAG = "PLANNER"; 

static trajectory_t _auto_traj = {0}; 
static bool _is_traj_initialized = false;


static void _Robot_Homing(robot_object_t *p_robot) {
    uint8_t homing_step = 50;
    theta_t theta_home = {0.0f, 0.0f, 0.0f}; // Home theta angles (can be adjusted depending on the robot configuration)

    point_t point_home = {.x = theta_home.arm_1, .y = theta_home.arm_1, .z = theta_home.arm_1}; // home theta
    point_t point_target; // stores the interpolation result

    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    theta_t theta_current = p_robot->theta_current;
    xSemaphoreGive(p_robot->lock); // Unlock after updating the flag

    point_t point_current = {.x = theta_current.arm_1, .y = theta_current.arm_1, .z = theta_current.arm_1}; // stores the current angles

    for (uint8_t i = 0; i <= homing_step; i++) {
        xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
        // Check the break homing flag before continuing interpolation
        if(p_robot->should_break_homing) {
            p_robot->should_break_homing = false; // Reset the flag after using it
            xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
            return; // Exit the homing process if the break flag is set
        }
        xSemaphoreGive(p_robot->lock); // Unlock after updating the flag


        // Linearly interpolate from theta_current back to theta_home
        point_target = Math_Linear_Interpolation(&point_current, &point_home, (float)i / (float)homing_step);

        theta_t theta_target = {.arm_1 = point_target.x, .arm_2 = point_target.y, .arm_3 = point_target.z};
        
        // ESP_LOGI(TAG, "Calculated HOME coordinates: X: %.2f | Y: %.2f | Z: %.2f", 
        //                       point_target.x, point_target.y, point_target.z);

        xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
        p_robot->theta_current = theta_target;
        p_robot->has_theta_current_changed = true; // Set this flag to true to indicate that the target theta angles have changed and need to be updated in the control system.
        xSemaphoreGive(p_robot->lock); // Unlock after updating the flag
        

        // === LOG HOME COORDINATES ===
        // ESP_LOGI(TAG, "Calculated HOME coordinates: MODE: %d | X: %.2f | Y: %.2f | Z: %.2f", 
        //                       point_target.mode, point_target.x, point_target.y, point_target.z);

        // Send the interpolated theta angles to the Control Task to drive the motors
        xQueueSend(g_queue_kinematics_to_control, &theta_target, portMAX_DELAY);

        // Pause for 20ms to allow the Kinematics Task to execute and avoid sending too fast
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void _Robot_Automatic(robot_object_t *p_robot) {
    if (!_is_traj_initialized) {
        _auto_traj.R = 130.0f;
        _auto_traj.z = -270.0f;
        _auto_traj.auto_state = 0;
        
        Start_Line(&_auto_traj, p_robot, _auto_traj.R, 0.0f, _auto_traj.z, 50);
                    
        _auto_traj.auto_state = 1;
        _is_traj_initialized = true;
    }

    point_t auto_point;
    bool has_next = Get_Next_Point(&_auto_traj, &auto_point);

    if (!has_next) {
        if (_auto_traj.auto_state == 1 || _auto_traj.auto_state == 2) {
            Start_Circle(&_auto_traj, 0.0f, 0.0f, _auto_traj.z, _auto_traj.R, 100);
            _auto_traj.auto_state = 2;
        }
        Get_Next_Point(&_auto_traj, &auto_point);
    }

    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    p_robot->end_effector_target = auto_point;
    p_robot->has_end_effector_target_changed = true; 
    xSemaphoreGive(p_robot->lock); // Unlock after updating the flag

    // === LOG HOME COORDINATES ===
    // ESP_LOGI(TAG, "Calculated Auto coordinates: MODE: %d | X: %.2f | Y: %.2f | Z: %.2f", 
    //                       auto_point.mode, auto_point.x, auto_point.y, auto_point.z);

    
    // Send the interpolated theta angles to the Kinematics Task to drive the motors
    xQueueSend(g_queue_planner_to_kinematics, &auto_point, portMAX_DELAY);

    // Pause for 20ms to allow the Kinematics Task to execute and avoid sending too fast
    vTaskDelay(pdMS_TO_TICKS(20));
}

static void _Robot_Manual(robot_object_t *p_robot, point_t *p_point_target) {
    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    point_t point_current = p_robot->end_effector_current; // Get the current end-effector point
    xSemaphoreGive(p_robot->lock); // Unlock after reading the current point

    Math_Low_Pass_Filter(&point_current, p_point_target); // Apply the low-pass filter to smooth the motion
    
    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    p_robot->end_effector_target = point_current;
    p_robot->has_end_effector_target_changed = true; 
    xSemaphoreGive(p_robot->lock); // Unlock after updating the flag

    // ESP_LOGI(TAG, "Calculated Manual coordinates: MODE: %d | X: %.2f | Y: %.2f | Z: %.2f", 
    //                       point_current.mode, point_current.x, point_current.y, point_current.z);

    // Send the interpolated theta angles to the Kinematics Task to drive the motors
    xQueueSend(g_queue_planner_to_kinematics, &point_current, portMAX_DELAY);
}

static void _Robot_Pick_And_Place(robot_object_t *p_robot, point_t *p_point_target) {
    if (p_robot == NULL || p_point_target == NULL) {
        return; 
    }

    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    point_t point_current = p_robot->end_effector_current; // Get the current end-effector point
    xSemaphoreGive(p_robot->lock); // Unlock after reading the current point

    if( point_current.x == p_point_target->x &&
        point_current.y == p_point_target->y &&
        point_current.z == p_point_target->z) 
        return; // If the target point is the same as the current point, there is no need to perform the pick and place process
    

    // Time and trajectory parameters for the pick and place process
    const uint16_t TOTAL_TIME_MS = 1000;    // Total movement time
    const uint16_t CYCLE_TIME_MS = 20;      // Interpolation cycle: 20ms (equivalent to 50Hz)
    const float CLEARANCE_HEIGHT = p_robot->Z_MAX - p_robot->Z_MIN - 1.0f;   // Object lifting height (parabola)

    const uint16_t TOTAL_STEPS = TOTAL_TIME_MS / CYCLE_TIME_MS; // Calculate the total number of interpolation steps based on total time and interpolation cycle

    // Initialize the time variable for vTaskDelayUntil
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CYCLE_TIME_MS);

    for (uint16_t i = 0; i <= TOTAL_STEPS; i++) {
        float t = (float)i / TOTAL_STEPS; 

        // Calculate and return the coordinates at step t
        point_t point_next = Math_Get_Parabolic_Arc_Point(&point_current, p_point_target, CLEARANCE_HEIGHT, t);

        xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
        p_robot->end_effector_target = point_next; // Update the target point    
        p_robot->has_end_effector_target_changed = true; // Set this flag to true to indicate that the target point has changed and needs to be updated in the control system.
        xSemaphoreGive(p_robot->lock); // Unlock after updating the flag

        // ESP_LOGI(TAG, "Calculated Pick and Place coordinates: MODE: %d | X: %.2f | Y: %.2f | Z: %.2f", 
        //                   point_next.mode, point_next.x, point_next.y, point_next.z);

        xQueueSend(g_queue_planner_to_kinematics, &point_next, portMAX_DELAY); // Send the interpolated point to the Kinematics Task to drive the motors

        // Real-time timing control
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


// =============================Planner Task=============================
void Robot_Planner_Task(void *pvParameters){
    point_t point_target = {0.0f, 0.0f, 0.0f, 0}; // stores the new point from the PC
    point_t point_current; // stores the previous point

    while (1) {
        //Check whether there are new coordinates from the computer (If not, use the old point)
        xQueueReceive(g_queue_udp_to_planner, &point_target, portMAX_DELAY);

        point_t point_current = point_target; // stores the previous point
        
        point_current.mode = point_target.mode; // Update the new mode

        if(point_current.mode == MODE_HOMING) {
            _Robot_Homing(g_p_robot); // Execute the homing process
        } 
        else if(point_current.mode == MODE_AUTOMATIC){
            _Robot_Automatic(g_p_robot); // execute the automatically generated trajectory process according to the scenario
        } 
        else if(point_current.mode == MODE_MANUAL){
            _Robot_Manual(g_p_robot, &point_target); // execute the manual control process using coordinates from the PC
        } 
        else if(point_current.mode == MODE_PICK_AND_PLACE){
            _Robot_Pick_And_Place(g_p_robot, &point_target); // execute the pick and place process using coordinates from the PC
        }
        //Pause for 20ms to allow other tasks on Core0 to run
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}