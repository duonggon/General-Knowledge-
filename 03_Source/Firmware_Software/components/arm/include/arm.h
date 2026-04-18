#pragma once

#include <stdint.h>
#include "driver/ledc.h" // Add the ESP-IDF hardware PWM library

typedef struct arm{
    uint8_t _pin; // GPIO pin used to control the motor
    ledc_channel_t _ledc_channel; // LEDC channel for PWM signal output
} arm_object_t;

void Arm_Init(arm_object_t *arm, uint8_t pin);