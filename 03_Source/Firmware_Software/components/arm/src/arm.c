#include "arm.h"

static void _Arm_Ledc_Channel_Init(arm_object_t *arm) {
    static ledc_channel_t _next_channel = LEDC_CHANNEL_0;
    arm->_ledc_channel = _next_channel;
    _next_channel = (ledc_channel_t)(_next_channel + 1);

    // Configure the channel to output pulses to the GPIO pin
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode     = LEDC_HIGH_SPEED_MODE;
    ledc_channel.channel        = arm->_ledc_channel;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num       = arm->_pin;
    ledc_channel.duty           = 0; // No pulse is output when it is first turned on
    ledc_channel.hpoint         = 0;
    ledc_channel_config(&ledc_channel);
}

void Arm_Init(arm_object_t *arm, uint8_t pin) {
    // Assign the GPIO pin to the arm
    arm->_pin = pin;
    
    // Configure the LEDC channel to output PWM pulses
    _Arm_Ledc_Channel_Init(arm);
}