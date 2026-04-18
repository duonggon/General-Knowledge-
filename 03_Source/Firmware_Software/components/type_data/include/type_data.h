#pragma once

#include <stdint.h>

// The core only contains 3D coordinates (x, y, z) for calculation and motor control
typedef struct {
    float x;
    float y;
    float z;
    uint8_t mode;
} point_t;

typedef struct {
    float arm_1;
    float arm_2;
    float arm_3;
} theta_t;