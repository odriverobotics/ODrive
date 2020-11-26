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


#include "oscilloscope.hpp"

// if you use the oscilloscope feature you can bump up this value
#define OSCILLOSCOPE_SIZE 4096

void Oscilloscope::update() {
    float trigger_data = trigger_src_ ? *trigger_src_ : 0.0f;
    float trigger_threshold = trigger_threshold_;
    float sample_data = data_src_ ? **data_src_ : 0.0f;

    if (trigger_data < trigger_threshold) {
        ready_ = true;
    }
    if (ready_ && trigger_data >= trigger_threshold) {
        capturing_ = true;
        ready_ = false;
    }
    if (capturing_) {
        if (pos_ < OSCILLOSCOPE_SIZE) {
            data_[pos_++] = sample_data;
        } else {
            pos_ = 0;
            capturing_ = false;
        }
    }
}
