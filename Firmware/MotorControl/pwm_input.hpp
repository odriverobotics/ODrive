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

#ifndef __PWM_INPUT_HPP
#define __PWM_INPUT_HPP

#include <tim.h>
#include <array>

class PwmInput {
public:
    PwmInput(TIM_HandleTypeDef* htim, std::array<uint16_t, 4> gpios)
            : htim_(htim), gpios_(gpios) {}

    void init();
    void on_capture();

private:
    void on_capture(int channel, uint32_t timestamp);

    TIM_HandleTypeDef* htim_;
    std::array<uint16_t, 4> gpios_;
};

#endif // __PWM_INPUT_HPP