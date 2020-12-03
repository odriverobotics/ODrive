#ifndef __TASK_TIMER_HPP
#define __TASK_TIMER_HPP

#include <stdint.h>
#include <board.h>

#define MEASURE_START_TIME
#define MEASURE_END_TIME
#define MEASURE_LENGTH
#define MEASURE_MAX_LENGTH

inline uint16_t sample_TIM13() {
    constexpr uint16_t clocks_per_cnt = (uint16_t)((float)TIM_1_8_CLOCK_HZ / (float)TIM_APB1_CLOCK_HZ);
    return clocks_per_cnt * TIM13->CNT;  // TODO: Use a hw_config
}

struct TaskTimer {
    uint32_t start_time_ = 0;
    uint32_t end_time_ = 0;
    uint32_t length_ = 0;
    uint32_t max_length_ = 0;

    static bool enabled;

    uint32_t start() {
        return sample_TIM13();
    }

    void stop(uint32_t start_time) {
        uint32_t end_time = sample_TIM13();
        uint32_t length = end_time - start_time;

        if (enabled) {
#ifdef MEASURE_START_TIME
            start_time_ = start_time;
#endif
#ifdef MEASURE_END_TIME
            end_time_ = end_time;
#endif
#ifdef MEASURE_LENGTH
            length_ = length;
#endif
        }
#ifdef MEASURE_MAX_LENGTH
        max_length_ = std::max(max_length_, length);
#endif
    }
};

struct TaskTimerContext {
    TaskTimerContext(const TaskTimerContext&) = delete;
    TaskTimerContext(const TaskTimerContext&&) = delete;
    void operator=(const TaskTimerContext&) = delete;
    void operator=(const TaskTimerContext&&) = delete;
    TaskTimerContext(TaskTimer& timer) : timer_(timer), start_time(timer.start()) {}
    ~TaskTimerContext() { timer_.stop(start_time); }
    
    TaskTimer& timer_;
    uint32_t start_time;
    bool exit_ = false;
};

#define MEASURE_TIME(timer) for (TaskTimerContext __task_timer_ctx{timer}; !__task_timer_ctx.exit_; __task_timer_ctx.exit_ = true)

#endif // __TASK_TIMER_HPP