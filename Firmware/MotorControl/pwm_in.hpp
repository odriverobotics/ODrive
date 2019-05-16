#ifndef __PWM_IN_HPP
#define __PWM_IN_HPP

#include "stm32_tim.hpp"

bool pwm_in_init(STM32_Timer_t* timer, uint8_t priority);

#endif // __PWM_IN_HPP