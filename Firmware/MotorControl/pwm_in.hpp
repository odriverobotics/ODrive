#ifndef __PWM_IN_HPP
#define __PWM_IN_HPP

#include "odrive_main.h"

bool pwm_in_init(STM32_Timer_t* timer, uint8_t priority);

#endif // __PWM_IN_HPP