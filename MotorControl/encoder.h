#pragma once

#include <drv8301.h>
#include <odrive_constants.h>

typedef struct
{
    TIM_HandleTypeDef *encoder_timer;
    int encoder_offset;
    int encoder_state;
    int motor_dir; // 1/-1 for fwd/rev alignment to encoder.
    float phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;
} Encoder_t;