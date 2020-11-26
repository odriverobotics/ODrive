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

#ifndef __OSCILLOSCOPE_HPP
#define __OSCILLOSCOPE_HPP

#include <autogen/interfaces.hpp>

// if you use the oscilloscope feature you can bump up this value
#define OSCILLOSCOPE_SIZE 4096

class Oscilloscope : public ODriveIntf::OscilloscopeIntf {
public:
    Oscilloscope(float* trigger_src, float trigger_threshold, float** data_src)
        : trigger_src_(trigger_src), trigger_threshold_(trigger_threshold), data_src_(data_src) {}

    float get_val(uint32_t index) override {
        return index < OSCILLOSCOPE_SIZE ? data_[index] : NAN;
    }

    void update();

    const uint32_t size_ = OSCILLOSCOPE_SIZE;
    const float* trigger_src_;
    const float trigger_threshold_;
    float* const * data_src_;

    float data_[OSCILLOSCOPE_SIZE] = {0};
    size_t pos_ = 0;
    bool ready_ = false;
    bool capturing_ = false;
};

#endif // __OSCILLOSCOPE_HPP