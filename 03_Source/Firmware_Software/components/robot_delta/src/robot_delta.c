#include "robot_delta.h"

#include "kinematics.h"

#include "esp_log.h" // Logging library


robot_object_t Robot_Create(const float A, const float RF, const float RE, const float Z_MIN, const float Z_MAX, const float R2){
    robot_object_t robot = {
        .A = A,
        .RF = RF,
        .RE = RE,
        .Z_MIN = Z_MIN,
        .Z_MAX = Z_MAX,
        .R2 = R2,

        .has_end_effector_current_changed = false, // by default, nothing has changed yet
        .has_theta_current_changed = false, // by default, nothing has changed yet

        .has_end_effector_target_changed = false, // by default, nothing has changed yet
        .has_theta_target_changed = false, // by default, nothing has changed yet

        .should_break_homing = false // by default, homing does not need to be interrupted
    };
    return robot;
}

static const char *TAG = "Control"; 

// This function runs when the ESP32 just starts up to set the initial point/angle
void Robot_Setup_Home_Point(robot_object_t *p_robot, theta_t *p_theta_home) {
    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    p_robot->has_theta_target_changed = true; // enable the flag for the first calculation at startup
    xSemaphoreGive(p_robot->lock); // Unlock after updating the flag


    point_t point_home = Kinematics_Call_Forward(p_robot, p_theta_home); // Calculate the home point from the home theta angles
    xSemaphoreTake(p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
    p_robot->end_effector_current = point_home;
    p_robot->theta_current = *p_theta_home;
    p_robot->has_end_effector_current_changed = false; 
    p_robot->has_theta_current_changed = false;
    xSemaphoreGive(p_robot->lock); // Unlock after updating the flag

    ESP_LOGI(TAG, "HOME point coordinates calculated: X: %.2f | Y: %.2f | Z: %.2f", 
                            point_home.x, point_home.y, point_home.z);
}