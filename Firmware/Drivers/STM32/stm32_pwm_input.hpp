#ifndef __STM32_PWM_INPUT_HPP
#define __STM32_PWM_INPUT_HPP

#include "stm32_gpio.hpp"
#include <tim.h>
#include <array>
#include <math.h>

class Stm32PwmInput {
public:
    Stm32PwmInput(TIM_HandleTypeDef* htim, std::array<Stm32Gpio, 4> gpios)
            : htim_(htim), gpios_(gpios) {}

    void init(std::array<bool, 4> enable_channels);
    void on_capture();
    TIM_TypeDef* get_timer() { return htim_->Instance; }

    TIM_HandleTypeDef* htim_;
    std::array<Stm32Gpio, 4> gpios_;
    float pwm_values_[4] = {NAN, NAN, NAN, NAN};

private:
    void handle_pulse(int channel, uint32_t high_time);
    void on_capture(int channel, uint32_t timestamp);

    uint32_t last_timestamp_[4] = { 0 };
    bool last_pin_state_[4] = { false };
    bool last_sample_valid_[4] = { false };
};

#endif // __STM32_PWM_INPUT_HPP