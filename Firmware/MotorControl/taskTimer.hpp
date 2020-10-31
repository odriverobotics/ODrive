#pragma once

#include <stdint.h>
#include <tim.h>

#include <algorithm>


inline uint16_t sample_TIM13() {
    constexpr uint16_t clocks_per_cnt = (uint16_t)((float)TIM_1_8_CLOCK_HZ / (float)TIM_APB1_CLOCK_HZ);
    return clocks_per_cnt * htim13.Instance->CNT;  // TODO: Use a hw_config
}

struct TaskTimer {
    uint32_t startTime = 0;
    uint32_t endTime = 0;
    uint32_t length = 0;
    uint32_t maxLength = 0;

    static bool sample_next;

    void beginTimer() {
        if (sample_next)
            startTime = sample_TIM13();
    }

    void stopTimer() {
        if (sample_next) {
            endTime = sample_TIM13();
            length = endTime - startTime;
            maxLength = std::max(maxLength, length);
        }
    }
};

extern volatile uint32_t adc_timestamp;