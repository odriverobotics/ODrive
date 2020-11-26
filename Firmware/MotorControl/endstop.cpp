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

#include <odrive_main.h>


void Endstop::update() {
    debounceTimer_.update();
    last_state_ = endstop_state_;
    if (config_.enabled) {
        bool last_pin_state = pin_state_;

        pin_state_ = get_gpio(config_.gpio_num).read();

        // If the pin state has changed, reset the timer
        if (pin_state_ != last_pin_state)
            debounceTimer_.reset();

        if (debounceTimer_.expired())
            endstop_state_ = config_.is_active_high ? pin_state_ : !pin_state_;  // endstop_state is the logical state
    } else {
        endstop_state_ = false;
    }
}

bool Endstop::apply_config() {
    debounceTimer_.reset();
    if (config_.enabled) {
        debounceTimer_.start();
    } else {
        debounceTimer_.stop();
    }
    debounceTimer_.setIncrement(config_.debounce_ms * 0.001f);
    return true;
}
