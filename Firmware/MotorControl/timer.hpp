/* MIT License

Copyright (c) 2016-2021 ODrive Robotics, Inc

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

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
