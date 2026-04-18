#include "app.h"
#include "wifi_app.h"
#include "over_the_air.h"
#include "globals.h"
#include "robot_delta.h"
#include "udp_receive.h"
#include "type_data.h"
#include "arm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Initialize the global variable that stores the robot state
robot_object_t *g_p_robot = NULL; // Initialize the robot pointer to NULL to ensure memory safety

// Define the Queues (necessary so the linker can find the allocated memory)
QueueHandle_t g_queue_udp_to_planner = NULL;
QueueHandle_t g_queue_planner_to_kinematics = NULL;
QueueHandle_t g_queue_kinematics_to_control = NULL;

TaskHandle_t g_handle_planner = NULL;
TaskHandle_t g_handle_kinematics = NULL;


// Function to initialize global variables
static void _App_Variables_Init() {
    // Allocate memory for the global variable that stores the robot state
    g_p_robot = (robot_object_t *)malloc(sizeof(robot_object_t));
    if (g_p_robot != NULL)
        // Initialize the global variable that stores the robot state
        *g_p_robot = Robot_Create(60.0f, 120.0f, 260.0f, -335.0f, -268.0, 135.0f);
    g_p_robot->lock = xSemaphoreCreateMutex(); // Create a mutex to protect access to the robot data

    // Assign GPIO pins to the 3 robot arms
    Arm_Init(&g_p_robot->_arm_1, ARM_1);
    Arm_Init(&g_p_robot->_arm_2, ARM_2);
    Arm_Init(&g_p_robot->_arm_3, ARM_3);

    // Initialize the Queues with appropriate sizes
    g_queue_udp_to_planner = xQueueCreate(1, sizeof(point_t));  
    g_queue_planner_to_kinematics = xQueueCreate(10, sizeof(point_t)); 
    g_queue_kinematics_to_control = xQueueCreate(5, sizeof(theta_t));
}

// Function to initialize tasks running in parallel on different cores
static void _App_Task_Init() {
    // =================================CORE 0=================================
    // Initialize the task that listens for UDP data from Python on Core 0
    xTaskCreatePinnedToCore(
        UDP_Receive_Task, // call the execution function in the task
        "UDP_Receive", 
        4096, 
        NULL, 
        5, // Higher priority (5)
        NULL, 
        0);

    xTaskCreatePinnedToCore(
        Robot_Planner_Task, // call the execution function in the task
        "Planner", 
        4096, 
        NULL, 
        4, // Lower priority (4)
        &g_handle_planner, 
        0);
    // suspend the Planner Task to wait for the Control_Servo Task to move the arms to the home position
    if (g_handle_planner != NULL) 
        vTaskSuspend(g_handle_planner);
    
    // Initialize the DNS spoofing task on Core 0 (OTA)
    xTaskCreatePinnedToCore(
        Wifi_DNS_Server_Task, // call the execution function in the task
        "DNS_Server", 
        2048, 
        NULL, 
        3, // Lower priority (3)
        NULL, 
        0);


    // =================================CORE 1=================================
    // Initialize the motor control task on Core 1
    xTaskCreatePinnedToCore(
        Robot_Motor_Control_Task, // call the execution function in the task
        "Control_Servo", 
        4096, 
        NULL, 
        6, 
        NULL, 
        1);

    // Initialize the Kinematics processing task on Core 1
    xTaskCreatePinnedToCore(
        Robot_Kinematics_Task, // call the execution function in the task
        "Kinematics", 
        8192, 
        NULL, 
        5, 
        &g_handle_kinematics, 
        1);
    // suspend the Kinematics Task to wait for the Control_Servo Task to move the arms to the home position
    if (g_handle_kinematics != NULL) 
        vTaskSuspend(g_handle_kinematics);
    
}

// Application initialization function
void App_Init() {
    // Initialize the Wifi Access Point with the name "Delta_Robot_v2", password "12345678", and allow a maximum of 1 connection
    Wifi_Init("Delta_Robot_v2", "12345678", 1);

    // Initialize the Web Server to listen for OTA upload connections
    OTA_Init_Web_Server();

     // Initialize global variables and Queues
    _App_Variables_Init();

    // Initialize parallel tasks
    _App_Task_Init();
}