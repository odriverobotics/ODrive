
#include "cmsis_event_loop.hpp"

bool CmsisEventLoop::init() {
    osMessageQDef(event_queue, kNCallbackSlots + kNTimerSlots, uint32_t);
    event_queue_ = osMessageCreate(osMessageQ(event_queue), NULL);
    return event_queue_ != nullptr;
}

bool CmsisEventLoop::run() {
    if (!event_queue_) {
        return false;
    }

    for (;;) {
        // Run callbacks of all overdue timers and find the delta T to the next
        // timer.
        uint32_t now = osKernelSysTick();
        int32_t smallest_delta_t = INT32_MAX;

        for (size_t i = 0; i < kNTimerSlots; ++i) {
            Timer& timer = timer_array_[i];
            if (timer.valid) {
                int32_t delta_ticks = (int32_t)(timer.trigger_time - now);
                if (delta_ticks <= 0) {
                    // Clear timer and free its associated memory slot
                    auto callback = timer.callback;
                    timer = {};
                    __atomic_or_fetch(&free_timers_, 1UL << i, __ATOMIC_SEQ_CST);

                    callback.invoke();
                } else {
                    smallest_delta_t = std::min(smallest_delta_t, delta_ticks);
                }
            }
        }

        // Await next event
        osEvent event = osMessageGet(event_queue_, smallest_delta_t);

        if (event.status != osEventMessage && event.status != osEventTimeout) {
            return false;
        }

        int slot = event.value.v;

        // If the loop was unblocked because a callback was posted, run the
        // callback.
        if (slot < kNCallbackSlots) {
            // Clear event and free its associated memory slot
            auto callback = callback_array_[slot];
            callback_array_[slot] = {};
            __atomic_or_fetch(&free_callbacks_, 1UL << slot, __ATOMIC_SEQ_CST);
            
            callback.invoke();
        }
    }

    return true;
}


bool CmsisEventLoop::put(fibre::Callback<void> callback) {
    // Lock-free allocation of one slot in callback_array_

    // Note: we could also use the mailbox API of CMSIS but that uses a critical
    // section to allocate a memory slot.

    int slot;
    uint32_t free_callbacks;
    do {
        free_callbacks = __atomic_load_n(&free_callbacks_, __ATOMIC_SEQ_CST);

        if (!(free_callbacks << (32 - kNCallbackSlots))) {
            return false; // All slots busy
        }

        slot = __builtin_ctz(free_callbacks); // count trailing zero bits
    } while (!__atomic_compare_exchange_n(&free_callbacks_, &free_callbacks, free_callbacks & ~(1UL << slot), true, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));

    callback_array_[slot] = callback;

    // Unblock the event loop thread
    return event_queue_ && (osMessagePut(event_queue_, slot, 0) == osOK);
}

bool CmsisEventLoop::call_later(float time, fibre::Callback<void> callback) {
    uint32_t delta_ticks = (uint32_t)(time * (float)osKernelSysTickFrequency);

    if (delta_ticks & 0x80000000) {
        return false; // Too large delta T
    }

    // Lock-free allocation of one slot in timer_array_

    int slot;
    uint32_t free_timers;
    do {
        free_timers = __atomic_load_n(&free_timers_, __ATOMIC_SEQ_CST);

        if (!(free_timers << (32 - kNTimerSlots))) {
            return false; // All slots busy
        }

        slot = __builtin_ctz(free_timers); // count trailing zero bits
    } while (!__atomic_compare_exchange_n(&free_timers_, &free_timers, free_timers & ~(1UL << slot), true, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));


    timer_array_[slot] = {
        .valid = false,
        .trigger_time = osKernelSysTick() + delta_ticks,
        .callback = callback,
    };
    portMEMORY_BARRIER();
    timer_array_[slot].valid = true;

    return unblock();
}

bool CmsisEventLoop::unblock() {
    return event_queue_ && (osMessagePut(event_queue_, kNCallbackSlots, 0) == osOK);
}
