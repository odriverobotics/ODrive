
#include <utils.hpp>
#include <board.h>


// @brief: Returns how much time is left until the deadline is reached.
// If the deadline has already passed, the return value is 0 (except if
// the deadline is very far in the past)
uint32_t deadline_to_timeout(uint32_t deadline_ms) {
    uint32_t now_ms = (uint32_t)((1000ull * (uint64_t)osKernelSysTick()) / osKernelSysTickFrequency);
    uint32_t timeout_ms = deadline_ms - now_ms;
    return (timeout_ms & 0x80000000) ? 0 : timeout_ms;
}

// @brief: Converts a timeout to a deadline based on the current time.
uint32_t timeout_to_deadline(uint32_t timeout_ms) {
    uint32_t now_ms = (uint32_t)((1000ull * (uint64_t)osKernelSysTick()) / osKernelSysTickFrequency);
    return now_ms + timeout_ms;
}

// @brief: Returns a non-zero value if the specified system time (in ms)
// is in the future or 0 otherwise.
// If the time lies far in the past this may falsely return a non-zero value.
int is_in_the_future(uint32_t time_ms) {
    return deadline_to_timeout(time_ms);
}

// @brief: Returns number of microseconds since system startup
uint32_t micros(void) {
    register uint32_t ms, cycle_cnt;
    do {
        ms = HAL_GetTick();
        cycle_cnt = TIM_TIME_BASE->CNT;
     } while (ms != HAL_GetTick());

    return (ms * 1000) + cycle_cnt;
}

// @brief: Busy wait delay for given amount of microseconds (us)
void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while (micros() - start < (uint32_t) us) {
        __ASM("nop");
    }
}
