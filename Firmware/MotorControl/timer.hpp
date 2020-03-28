#pragma once

#include <algorithm>
template <class T>
class Timer {
   public:
    void setTimeout(const T timeout) {
        timeout_ = timeout;
    }

    void setIncrement(const T increment) {
        increment_ = increment;
    }

    void start() {
        running_ = true;
    }

    void stop() {
        running_ = false;
    }

    // If the timer is started, increment the timer
    void update() {
        if (running_)
            timer_ = std::min<T>(timer_ + increment_, timeout_);
    }

    void reset() {
        timer_ = static_cast<T>(0);
    }

    bool expired() {
        return timer_ >= timeout_;
    }

   private:
    T timer_ = static_cast<T>(0);     // Current state
    T timeout_ = static_cast<T>(0);   // Time to count
    T increment_ = static_cast<T>(0);  // Amount to increment each time update() is called
    bool running_ = false;            // update() only increments if runing_ is true
};
