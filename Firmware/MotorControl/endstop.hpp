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

#ifndef __ENDSTOP_HPP
#define __ENDSTOP_HPP

#include "timer.hpp"
class Endstop {
   public:
    struct Config_t {
        float offset = 0;
        uint32_t debounce_ms = 50;
        uint16_t gpio_num = 0;
        bool enabled = false;
        bool is_active_high = false;

        // custom setters
        Endstop* parent = nullptr;
        void set_gpio_num(uint16_t value) { gpio_num = value; parent->apply_config(); }
        void set_enabled(uint32_t value) { enabled = value; parent->apply_config(); }
        void set_debounce_ms(uint32_t value) { debounce_ms = value; parent->apply_config(); }
    };

    Endstop() {}

    Endstop::Config_t config_;
    Axis* axis_ = nullptr;

    bool apply_config();

    void update();
    constexpr bool get_state(){
        return endstop_state_;
    }

    constexpr bool rose(){
        return (endstop_state_ != last_state_) && endstop_state_;
    }

    constexpr bool fell(){
        return (endstop_state_ != last_state_) && !endstop_state_;
    }

    bool endstop_state_ = false;

   private:
    bool last_state_ = false;
    bool pin_state_ = false;
    float pos_when_pressed_ = 0.0f;
    Timer<float> debounceTimer_;
};
#endif