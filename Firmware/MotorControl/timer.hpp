#pragma once

#include <algorithm>
class Timer {
   public:
    void setTimeout(const float timeout) {
        timeout_ = timeout;
    }

    void setInterval(const float interval) {
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
        timer_ = 0.0f;
    }

    bool expired() {
        return timer_ >= timeout_;
    }

   private:
    float timer_ = 0.0f;
    float timeout_ = 0.0f;
    float interval_ = 0.0f;
    bool running_ = false;
};
