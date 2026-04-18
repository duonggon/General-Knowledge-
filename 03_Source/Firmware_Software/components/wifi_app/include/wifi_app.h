#pragma once
#include <stdio.h>

// Function to initialize the ESP32 as a Wi-Fi Access Point (WiFi name, password, maximum allowed connections)
void Wifi_Init(const char* WIFI_AP_SSID, const char* WIFI_AP_PASS, uint8_t MAX_STA_CONN);

// DNS Server task to trick devices
void Wifi_DNS_Server_Task(void *pvParameters);