#ifndef __FREERTOS_HPP
#define __FREERTOS_HPP

#include <stm32_tim.hpp>

typedef int (*main_task_t)();

// TODO: use generic timer type
bool freertos_init(STM32_Timer_t* system_timer, main_task_t main_task);

#endif //  __FREERTOS_HPP