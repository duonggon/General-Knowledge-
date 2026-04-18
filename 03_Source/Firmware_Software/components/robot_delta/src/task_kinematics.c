#include "task_kinematics.h"

#include "kinematics.h"
#include "math_until.h"

#include "esp_log.h" // Logging library

// Define a name (TAG) for easier log filtering in the terminal
static const char *TAG = "Kinematic"; 


// ==============================Inverse Kinematics Calculation Task===============================
void Robot_Kinematics_Task(void *pvParameters){
    point_t point_target;
    theta_t theta_target; // stores the calculated target theta angles from the target point
    while (1) {
        // Check whether a new target point has been sent from the planner to kinematics; otherwise put this task to sleep
        if (xQueueReceive(g_queue_planner_to_kinematics, &point_target, portMAX_DELAY)) {
            
            theta_target = Kinematics_Call_Inverse(g_p_robot, &point_target); // Calculate the target theta angles from the target point and update them in the robot struct
            
            xQueueSend(g_queue_kinematics_to_control, &theta_target, portMAX_DELAY); // Send the calculated target theta angles back to the planner so the planner can use them during motion planning.

            // ESP_LOGI(TAG, "Calculated angles (rad): Arm 1: %.2f | Arm 2: %.2f | Arm 3: %.2f", 
            //               theta_target.arm_1, theta_target.arm_2, theta_target.arm_3);

            
            xSemaphoreTake(g_p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
            g_p_robot->end_effector_current = point_target; // Update the current point in the Robot
            g_p_robot->has_end_effector_current_changed = true; // Set the flag to indicate that the current point has changed
            g_p_robot->theta_current = theta_target; // Update the current point in the Robot
            g_p_robot->has_theta_current_changed = true; // Set the flag to indicate that the current angle has changed
            xSemaphoreGive(g_p_robot->lock); // Unlock after reading the current point
        }
    }
}