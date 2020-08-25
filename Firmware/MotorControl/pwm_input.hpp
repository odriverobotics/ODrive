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