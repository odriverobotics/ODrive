#pragma once

#include <algorithm>
template <class T>
class Timer {
   public:
    void setTimeout(const T timeout) {
        timeout_ = timeout;
    }

    void setInterval(const T interval) {
        interval_ = interval;
    }

    void start() {
        running_ = true;
    }

    void stop() {
        running_ = false;
    }

    void update() {
        if (running_)
            timer_ = std::min(timer_ + interval_, timeout_);
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
    T interval_ = static_cast<T>(0);  // Amount to increment each time update() is called
    bool running_ = false;            // update() only increments if runing_ is true
};
