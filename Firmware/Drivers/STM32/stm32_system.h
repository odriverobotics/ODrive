#ifndef __STM32_SYSTEM_H
#define __STM32_SYSTEM_H

#if defined(STM32F405xx)
#include <stm32f405xx.h>
#else
#error "unknown STM32 microcontroller"
#endif

// C/C++ definitions

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t cpu_enter_critical() {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void cpu_exit_critical(uint32_t priority_mask) {
    __set_PRIMASK(priority_mask);
}

#ifdef __cplusplus
}
#endif


// C++ only definitions

#ifdef __cplusplus

struct CriticalSectionContext {
    CriticalSectionContext(const CriticalSectionContext&) = delete;
    CriticalSectionContext(const CriticalSectionContext&&) = delete;
    void operator=(const CriticalSectionContext&) = delete;
    void operator=(const CriticalSectionContext&&) = delete;
    CriticalSectionContext() : mask_(cpu_enter_critical()) {}
    ~CriticalSectionContext() { cpu_exit_critical(mask_); }
    uint32_t mask_;
    bool exit_ = false;
};

#define CRITICAL_SECTION() for (CriticalSectionContext __critical_section_context; !__critical_section_context.exit_; __critical_section_context.exit_ = true)

#endif

#endif // __STM32_SYSTEM_H