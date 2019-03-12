#ifndef __STM32_SYSTEM_HPP
#define __STM32_SYSTEM_HPP

#include <stdbool.h>
#include "stm32f4xx_hal.h"

//TODO: this should come from FreeRTOS files
#define TIM_TIME_BASE TIM14


#ifdef __cplusplus
extern "C" {
#endif

void _Error_Handler(char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

extern uint32_t _reboot_cookie;

/**
 * @brief Initializes low level system features such as clocks, fault handlers
 * and memories.
 */
bool system_init(void);

/**
 * @brief: Starts a section that cannot be interrupted by any normal interrupt.
 */
static inline uint32_t cpu_enter_critical() {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief: When called with the return value of cpu_enter_critical(), the
 * interrupts are re-enabled if and only if they were enabled prior to entering
 * the critical section.
 */
static inline void cpu_exit_critical(uint32_t priority_mask) {
    __set_PRIMASK(priority_mask);
}

/** @brief: Returns number of microseconds since system startup */
static inline uint32_t micros(void) {
    register uint32_t ms, cycle_cnt;
    do {
        ms = HAL_GetTick();
        cycle_cnt = TIM_TIME_BASE->CNT;
     } while (ms != HAL_GetTick());

    return (ms * 1000) + cycle_cnt;
}

// @brief: Busy wait delay for given amount of microseconds (us)
static inline void delay_us(uint32_t us) {
    uint32_t start = micros();
    while (micros() - start < (uint32_t) us) {
        __ASM("nop");
    }
}

#ifdef __cplusplus
}
#endif

#endif // __STM32_SYSTEM_HPP