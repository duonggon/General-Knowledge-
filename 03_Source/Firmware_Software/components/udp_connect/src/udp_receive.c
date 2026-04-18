#include "udp_receive.h"
#include "globals.h"
#include "type_data.h"
#include "robot_delta.h"
#include "lwip/sockets.h"
#include "esp_log.h" // Logging library
#include <string.h>

#define UDP_PORT 1234  // UDP port for Python to send data to

// Define a name (TAG) for easier log filtering in the terminal
static const char *TAG = "UDP_RX"; 

void UDP_Receive_Task(void *pvParameters) {  
    char rx_buffer[128];
    point_t target;

    // Clear garbage data from RAM
    memset(&target, 0, sizeof(point_t));
    
    while (true) {
        // Configure UDP socket
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(UDP_PORT);

        // Create socket and bind
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

        // If socket creation fails, wait 0.5s and try again
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue; // Repeat the outer while loop
        }

        // Check bind error; if it fails, close the socket and recreate it from scratch
        if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(500)); // REQUIRED
            continue;
        }
        
        // Set receive timeout to avoid task hanging if the PC sends nothing
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 20000; // Set a 20ms alarm
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        // ====================================================================

        // Prepare to receive data
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);

        while (true) {
            // This function will automatically return after 20ms if the PC sends nothing
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            
            if (len > 0) {
                rx_buffer[len] = 0; // Terminate the string for parsing
                
                // === LOG THE RAW RECEIVED STRING ===
                // ESP_LOGI(TAG, "Received string from PC: %s", rx_buffer);

                // Start parsing rx_buffer using commas
                char* ptr = strtok(rx_buffer, ",");
                if (ptr != NULL) {
                    target.mode = atoi(ptr); 
                    
                    ptr = strtok(NULL, ",");
                    if (ptr != NULL) target.x = strtof(ptr, NULL);
                    
                    ptr = strtok(NULL, ",");
                    if (ptr != NULL) target.y = strtof(ptr, NULL);
                    
                    ptr = strtok(NULL, ",");
                    if (ptr != NULL) target.z = strtof(ptr, NULL);

                    // === LOG THE RESULT AFTER PARSING THE STRING ===
                    // ESP_LOGI(TAG, "Decoded data -> Mode: %d | X: %.2f | Y: %.2f | Z: %.2f", 
                    //          target.mode, target.x, target.y, target.z);

                    // Set the HOMING interrupt flag
                    if(target.mode == MODE_HOMING) {
                        xSemaphoreTake(g_p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
                        g_p_robot->should_break_homing = false; // Reset the flag after using it
                        xSemaphoreGive(g_p_robot->lock); // Unlock after updating the flag
                    } else {
                        xSemaphoreTake(g_p_robot->lock, portMAX_DELAY); // Lock to ensure safe access to the robot
                        g_p_robot->should_break_homing = true; // Set the break homing flag so if homing is in progress, it stops immediately
                        xSemaphoreGive(g_p_robot->lock); // Unlock after updating the flag
                    }

                    xQueueOverwrite(g_queue_udp_to_planner, &target);
                }
            }
            else if (len < 0) {
                // Distinguish between "Timeout" and "Network error"
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == 11) {
                    // This error means 20ms passed and the PC still sent nothing.
                    continue; 
                } else {
                    // This is a serious error (Wi-Fi dropped, socket broken)
                    ESP_LOGW(TAG, "Network error or connection lost, reinitializing Socket...");
                    break; // exit the inner loop to close the socket and recreate it from scratch
                }
            }
        }
        // Close the socket
        close(sock);
    }
}