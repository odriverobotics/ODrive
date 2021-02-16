#ifndef __CMSIS_EVENT_LOOP_HPP
#define __CMSIS_EVENT_LOOP_HPP

#include <fibre/callback.hpp>
#include <cmsis_os.h>

class CmsisEventLoop {
public:
    static constexpr int kNCallbackSlots = 32;
    static constexpr int kNTimerSlots = 32;

    bool init();
    bool run();
    bool put(fibre::Callback<void> callback);
    bool call_later(float time, fibre::Callback<void> callback);

private:
    struct Timer {
        bool valid = false;
        uint32_t trigger_time;
        fibre::Callback<void> callback;
    };

    bool unblock();

    std::array<fibre::Callback<void>, kNCallbackSlots> callback_array_;
    uint32_t free_callbacks_ = 0xffffffff;
    static_assert(kNCallbackSlots <= 32); // bit field represents at most 32 slots

    std::array<Timer, kNTimerSlots> timer_array_;
    uint32_t free_timers_ = 0xffffffff;
    static_assert(kNTimerSlots <= 32); // bit field represents at most 32 slots

    osMessageQId event_queue_ = nullptr;
};

#endif // __CMSIS_EVENT_LOOP_HPP
