#ifndef __STM32_SYSTEM_HPP
#define __STM32_SYSTEM_HPP

#include <stdbool.h>
#include "stm32f4xx_hal.h"


#ifdef __cplusplus
extern "C" {
#endif

#ifdef STM32F4
typedef struct {
    __IO uint32_t ISR;   /*!< DMA interrupt status register */
    __IO uint32_t Reserved0;
    __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;
#endif

void _Error_Handler(char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

extern uint32_t _reboot_cookie;

// Monotonic clock variables
#define TICK_TIMER_PERIOD_BITS  16
#define TICK_TIMER_PERIOD (1 << TICK_TIMER_PERIOD_BITS) // must be a power of 2
extern volatile uint32_t* tick_timer_cnt;
extern volatile uint64_t ticks_us;
extern volatile uint16_t tick_timer_last_cnt;

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

static inline uint64_t get_ticks_us() {
    if (tick_timer_cnt) {
        uint32_t mask = cpu_enter_critical();
        uint16_t new_cnt = *tick_timer_cnt;
        uint16_t delta = (new_cnt - tick_timer_last_cnt) & (TICK_TIMER_PERIOD - 1);
        ticks_us += delta;
        tick_timer_last_cnt = new_cnt;
        cpu_exit_critical(mask);
    }

    return ticks_us;
}

static inline uint32_t get_ticks_ms() {
    return get_ticks_us() / 1000ULL;
}

/** @brief: Returns number of microseconds since system startup */
static inline uint32_t micros(void) {
    return (uint32_t) get_ticks_us();
}

/** @brief: Busy wait delay for given amount of microseconds (us) */
static inline void delay_us(uint32_t us) {
    uint32_t start = micros();
    while (micros() - start < (uint32_t) us) {
        __ASM("nop");
    }
}

static inline void enable_interrupt(IRQn_Type irqn, uint8_t priority) {
    // TODO: do reference counting and ensure that the priority is not changed when already enabled
    NVIC_SetPriority(irqn, priority >> __NVIC_PRIO_BITS);
    HAL_NVIC_EnableIRQ(irqn);
}

static inline bool get_interrupt_enabled(IRQn_Type IRQn) {
    if (IRQn < 0)
        return true;
    else
        return NVIC->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}

void dump_interrupts(bool show_enabled_only);

#ifdef __cplusplus
} // extern "C"

/* C++ declarations ----------------------------------------------------------*/

#include "stm32_tim.hpp"

class PerformanceCounter_t {
public:
    PerformanceCounter_t(volatile uint32_t& var)
            : start_(DWT->CYCCNT), upper_counter_(current_counter), var_(var) {
        // Fun fact: there's a race condition here.
        current_counter = this;
    }

    ~PerformanceCounter_t() {
        uint32_t mask = cpu_enter_critical();
        uint32_t cnt = DWT->CYCCNT;
        uint32_t subtract = interrupted_;
        cpu_exit_critical(mask);
        uint32_t total_time = cnt - start_;
        var_ += total_time - subtract;
        if (upper_counter_) {
            upper_counter_->interrupted_ += total_time;
        }
        current_counter = upper_counter_;
    }
private:
    uint32_t start_;
    volatile uint32_t interrupted_ = 0;
    PerformanceCounter_t* upper_counter_;
    volatile uint32_t& var_;
    static PerformanceCounter_t* current_counter;
};

/**
 * @brief Uses the given timer to initialize a monotonic 64 bit microsecond
 * resolution clock.
 * 
 * The current clock value can be accessed through get_ticks_us() and
 * get_ticks_ms().
 * 
 * This will enable an interrupt with `tick_timer_priority` that fires roughly
 * every ~25ms. If the interrupt handler is delayed by more than ~25ms the clock
 * value becomes inaccurate.
 */
bool init_monotonic_clock(STM32_Timer_t* tick_timer, uint8_t tick_timer_priority);

#endif

#endif // __STM32_SYSTEM_HPP