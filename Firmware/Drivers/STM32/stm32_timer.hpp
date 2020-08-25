#ifndef __STM32_TIMER_HPP
#define __STM32_TIMER_HPP

#include "stm32_system.h"
#include <tim.h>
#include <array>

class Stm32Timer {
public:
    /**
     * @brief Starts multiple timers deterministically and synchronously from the
     * specified offset.
     * 
     * All timers are atomically (*) put into the following state (regardless of
     * their previous state/configuration):
     *  - TIMx_CNT will be initialized according to the corresponding counter[i] parameter.
     *  - If the timer is in center-aligned mode, it will be set to up-counting direction.
     *  - The update repetition counter is reset to TIMx_RCR (if applicable).
     *  - The prescaler counter is reset.
     *  - Update interrupts are disabled.
     *  - The counter put into running state.
     * 
     * This function is implemented by generating an update event on all selected timers.
     * That means as a side effect all things that are connected to the update event
     * except the interrupt routine itself (i.e. ADCs, DMAs, slave timers, etc) will
     * be triggered.
     * 
     * Also you probably want to disable any connected PWM outputs to prevent glitches.
     * 
     * (*) Best-effort atomically. There will be skew of a handful of clock cycles
     *     but it's always the same given the compiler version and configuration.
     */
    template<size_t I>
    static void start_synchronously(std::array<TIM_HandleTypeDef*, I> timers, std::array<size_t, I> counters) {
        start_synchronously_impl(timers, counters, std::make_index_sequence<I>());
    }

private:

#pragma GCC push_options
#pragma GCC optimize (3)

    template<size_t I, size_t ... Is>
    static void start_synchronously_impl(std::array<TIM_HandleTypeDef*, I> timers, std::array<size_t, I> counters, std::index_sequence<Is...>) {
        for (size_t i = 0; i < I; ++i) {
            TIM_HandleTypeDef* htim = timers[i];

            // Stop the timer so we can start all of them later more atomically.
            htim->Instance->CR1 &= ~TIM_CR1_CEN;

            // Generate update event to force all of the timer's registers into
            // a known state.
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
            htim->Instance->EGR |= TIM_EGR_UG;
            __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);

            // Load counter with the desired value.
            htim->Instance->CNT = counters[i];
        }

        register volatile uint32_t* cr_addr[I];
        register uint32_t cr_val[I];
        for (size_t i = 0; i < I; ++i) {
            cr_addr[i] = &timers[i]->Instance->CR1;
            cr_val[i] = timers[i]->Instance->CR1 | TIM_CR1_CEN;
        }

        // Restart all timers as atomically as possible.
        // By inspection we find that this is compiled to the following code:
        //    f7ff faa0 	bl	800bdd0 <cpu_enter_critical()>
        //    f8c9 6000 	str.w	r6, [r9]
        //    f8c8 5000 	str.w	r5, [r8]
        //    603c      	str	r4, [r7, #0]
        //    f7ff fa9d 	bl	800bdd8 <cpu_exit_critical(unsigned long)>
        uint32_t mask = cpu_enter_critical();
        int dummy[I] = {(*cr_addr[Is] = cr_val[Is], 0)...};
        (void)dummy;
        cpu_exit_critical(mask);
    }
#pragma GCC pop_options
};

#endif // __STM32_TIMER_HPP