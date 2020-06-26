#ifndef __STM32_SYSTEM_H
#define __STM32_SYSTEM_H

#include <cmsis_os.h>

static inline uint32_t cpu_enter_critical() {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void cpu_exit_critical(uint32_t priority_mask) {
    __set_PRIMASK(priority_mask);
}

#endif // __STM32_SYSTEM_H