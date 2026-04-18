#include "task_motor_control.h"

#include "arm.h"
#include "globals.h"
#include "type_data.h"
#include "arm.h"
#include "robot_delta.h"
#include "kinematics.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>


// PWM settings for MG90S (14-bit resolution)
// 14-bit means one 20ms cycle is divided into 16384 steps
#define DUTY_MIN 410  // Equivalent to a 0.5ms pulse (0-degree angle)
#define DUTY_MAX 1966 // Equivalent to a 2.0ms pulse (180-degree angle)


static void _Robot_Timer_Init() {
    // Frequency 50Hz, 14-bit resolution
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_0; // 3 arms share the same Timer
    ledc_timer.duty_resolution  = LEDC_TIMER_14_BIT;
    ledc_timer.freq_hz          = 50;           // Standard frequency of MG996R (50Hz)
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);
}

// Function to calculate the output angle for one arm
static float _Robot_Calculate_Servo_Angle(float angle_rad) {
    // Convert to Degrees
    float angle_deg = angle_rad * 180.0f / M_PI;

    // Mechanical angle compensation (Map IK angle 0 to Servo center 90)
    float servo_angle = 90.0f - angle_deg;

    // Mechanical safety clamp (Prevent servo gear damage)
    if (servo_angle < 0.0f) return 0.0f;
    if (servo_angle > 180.0f) return 180.0f;
    
    return servo_angle;
}

// Pulse mapping (Degree -> Duty)
static uint32_t _Robot_Calculate_Duty(float servo_angle) {
    // Linearly map the angle range (0 - 180) to the pulse range (410 - 1966)
    return DUTY_MIN + (uint32_t)((servo_angle / 180.0f) * (DUTY_MAX - DUTY_MIN));
}

// Hardware interface (Duty -> LEDC)
static void _Robot_Set_PWWM_Duty(arm_object_t *arm, uint32_t duty) {
    // Push the signal into the ESP32 hardware register
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, arm->_ledc_channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, arm->_ledc_channel);
}

static void _Robot_Write_PIN(arm_object_t *arm, float angle_rad) {
    float safe_angle = _Robot_Calculate_Servo_Angle(angle_rad);
    uint32_t duty = _Robot_Calculate_Duty(safe_angle);
    _Robot_Set_PWWM_Duty(arm, duty);
}

static void _Robot_Write_All_Pins(robot_object_t *p_robot, theta_t *p_theta) {
    _Robot_Write_PIN(&p_robot->_arm_1, p_theta->arm_1);
    _Robot_Write_PIN(&p_robot->_arm_2, p_theta->arm_2);
    _Robot_Write_PIN(&p_robot->_arm_3, p_theta->arm_3);
}



// ================================ Motor Control Task =================================
void Robot_Motor_Control_Task(void *pvParameters){
    // Initialize the Timer for PWM output
    _Robot_Timer_Init();

    theta_t theta_target = {0, 0, 0};
    Robot_Setup_Home_Point(g_p_robot, &theta_target); // Move the robot variable to the initial home point

    // Move the arms to Home
    _Robot_Write_All_Pins(g_p_robot, &theta_target);

    // stop for a while to ensure the arms have returned to the home position before allowing the other tasks to run
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds

    // Allow the previously suspended tasks to run
    if (g_handle_planner != NULL)
        vTaskResume(g_handle_planner);
    if (g_handle_kinematics != NULL)
        vTaskResume(g_handle_kinematics);

    // main execution loop of the motor control task
    while (1) {
        // Check whether a new target angle set has been sent from kinematics to control; otherwise put this task to sleep
        if (xQueueReceive(g_queue_kinematics_to_control, &theta_target, portMAX_DELAY)) {
        _Robot_Write_All_Pins(g_p_robot, &theta_target);     
        }
    }
}