#include "encoder.h"

Encoder::Encoder(TIM_HandleTypeDef *encoder_timer){
    this->encoder_timer = encoder_timer;
    this->encoder_offset = 0;
    this->encoder_state = 0;
    this->motor_dir = 0;
    this->phase = 0.0f;
    this->pll_pos = 0.0f;
    this->pll_vel = 0.0f;
    this->pll_kp = 0.0f;
    this->pll_ki = 0.0f;
}