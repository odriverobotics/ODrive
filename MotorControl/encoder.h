#pragma once

#include <drv8301.h>

class Encoder {
  public:
    Encoder(TIM_HandleTypeDef *encoder_timer);

    // TODO:  Get/Set functions as needed
    TIM_HandleTypeDef *encoder_timer;
    int encoder_offset;
    int encoder_state;
    int motor_dir;  // 1/-1 for fwd/rev alignment to encoder.
    float phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;

    // TODO:  All functions which operate on an Encoder should be placed here
};