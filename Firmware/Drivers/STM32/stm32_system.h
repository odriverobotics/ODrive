#ifndef __STM32_SYSTEM_H
#define __STM32_SYSTEM_H

#if defined(STM32F405xx)
#include <stm32f405xx.h>
#elif defined(STM32F722xx)
#include <stm32f722xx.h>
#else
#error "unknown STM32 microcontroller"
#endif

// C/C++ definitions

#ifdef __cplusplus
extern "C" {
#endif

// Uncomment the following line to sacrifice 1kB of RAM for the ability to
// monitor the number of times each interrupt fires.
//#define ENABLE_IRQ_COUNTER

#ifdef ENABLE_IRQ_COUNTER
extern uint32_t irq_counters[];
#define COUNT_IRQ(irqn) (++irq_counters[irqn + 14])
#define GET_IRQ_COUNTER(irqn) irq_counters[irqn + 14]
#else
#define COUNT_IRQ(irqn) ((void)0)
#define GET_IRQ_COUNTER(irqn) 0
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
    operator bool() { return true; };
    CriticalSectionContext() : mask_(cpu_enter_critical()) {}
    ~CriticalSectionContext() { cpu_exit_critical(mask_); }
    uint32_t mask_;
    bool exit_ = false;
};

#ifdef __clang__
#define CRITICAL_SECTION() for (CriticalSectionContext __critical_section_context; !__critical_section_context.exit_; __critical_section_context.exit_ = true)
#else
#define CRITICAL_SECTION() if (CriticalSectionContext __critical_section_context{})
#endif

#endif

#endif // __STM32_SYSTEM_H