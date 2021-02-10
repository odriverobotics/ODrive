
#include "stm32_pwm_input.hpp"
#include "stm32_timer.hpp"
#include <algorithm>

void Stm32PwmInput::init(std::array<bool, 4> enable_channels) {
    Stm32Timer{htim_->Instance}.enable_clock();

    TIM_IC_InitTypeDef sConfigIC;
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;

    uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

    for (size_t i = 0; i < 4; ++i) {
        if (enable_channels[i]) {
            HAL_TIM_IC_ConfigChannel(htim_, &sConfigIC, channels[i]);
            HAL_TIM_IC_Start_IT(htim_, channels[i]);
        }
    }
}

//TODO: These expressions have integer division by 1MHz, so it will be incorrect for clock speeds of not-integer MHz
#define TIM_2_5_CLOCK_HZ        TIM_APB1_CLOCK_HZ
#define PWM_MIN_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 1000UL) // 1ms high is considered full reverse
#define PWM_MAX_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2000UL) // 2ms high is considered full forward
#define PWM_MIN_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 500UL) // ignore high periods shorter than 0.5ms
#define PWM_MAX_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2500UL) // ignore high periods longer than 2.5ms
#define PWM_INVERT_INPUT        false

/**
 * @param channel: A channel number in [0, 3]
 */
void Stm32PwmInput::handle_pulse(int channel, uint32_t high_time) {
    if (high_time < PWM_MIN_LEGAL_HIGH_TIME || high_time > PWM_MAX_LEGAL_HIGH_TIME)
        return;

    high_time = std::clamp(high_time, PWM_MIN_HIGH_TIME, PWM_MAX_HIGH_TIME);
    float fraction = (float)(high_time - PWM_MIN_HIGH_TIME) / (float)(PWM_MAX_HIGH_TIME - PWM_MIN_HIGH_TIME);

    pwm_values_[channel] = fraction;
}

/**
 * @param channel: A channel number in [0, 3]
 */
void Stm32PwmInput::on_capture(int channel, uint32_t timestamp) {
    if (channel >= 4)
        return;
    Stm32Gpio gpio = gpios_[channel];
    if (!gpio)
        return;
    bool current_pin_state = gpio.read();

    if (last_sample_valid_[channel]
        && (last_pin_state_[channel] != PWM_INVERT_INPUT)
        && (current_pin_state == PWM_INVERT_INPUT)) {
        handle_pulse(channel, timestamp - last_timestamp_[channel]);
    }

    last_timestamp_[channel] = timestamp;
    last_pin_state_[channel] = current_pin_state;
    last_sample_valid_[channel] = true;
}

void Stm32PwmInput::on_capture() {
    if(__HAL_TIM_GET_FLAG(htim_, TIM_FLAG_CC1)) {
        __HAL_TIM_CLEAR_IT(htim_, TIM_IT_CC1);
        on_capture(0, htim_->Instance->CCR1);
    }
    if(__HAL_TIM_GET_FLAG(htim_, TIM_FLAG_CC2)) {
        __HAL_TIM_CLEAR_IT(htim_, TIM_IT_CC2);
        on_capture(1, htim_->Instance->CCR2);
    }
    if(__HAL_TIM_GET_FLAG(htim_, TIM_FLAG_CC3)) {
        __HAL_TIM_CLEAR_IT(htim_, TIM_IT_CC3);
        on_capture(2, htim_->Instance->CCR3);
    }
    if(__HAL_TIM_GET_FLAG(htim_, TIM_FLAG_CC4)) {
        __HAL_TIM_CLEAR_IT(htim_, TIM_IT_CC4);
        on_capture(3, htim_->Instance->CCR4);
    }
}
