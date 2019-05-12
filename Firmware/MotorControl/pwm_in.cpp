
#include "pwm_in.hpp"

//TODO: These expressions have integer division by 1MHz, so it will be incorrect for clock speeds of not-integer MHz
#define TIM_2_5_CLOCK_HZ        TIM_APB1_CLOCK_HZ
#define PWM_MIN_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 1000UL) // 1ms high is considered full reverse
#define PWM_MAX_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2000UL) // 2ms high is considered full forward
#define PWM_MIN_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 500UL) // ignore high periods shorter than 0.5ms
#define PWM_MAX_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2500UL) // ignore high periods longer than 2.5ms
#define PWM_INVERT_INPUT        false

static void handle_pulse(int gpio_num, uint32_t high_time) {
    if (high_time < PWM_MIN_LEGAL_HIGH_TIME || high_time > PWM_MAX_LEGAL_HIGH_TIME)
        return;

    if (high_time < PWM_MIN_HIGH_TIME)
        high_time = PWM_MIN_HIGH_TIME;
    if (high_time > PWM_MAX_HIGH_TIME)
        high_time = PWM_MAX_HIGH_TIME;
    float fraction = (float)(high_time - PWM_MIN_HIGH_TIME) / (float)(PWM_MAX_HIGH_TIME - PWM_MIN_HIGH_TIME);
    float value = board_config.pwm_mappings[gpio_num - 1].min +
                  (fraction * (board_config.pwm_mappings[gpio_num - 1].max - board_config.pwm_mappings[gpio_num - 1].min));

    Endpoint* endpoint = get_endpoint(board_config.pwm_mappings[gpio_num - 1].endpoint);
    if (!endpoint)
        return;

    endpoint->set_from_float(value);
}

static void pwm_in_cb(void* ctx, uint32_t channel, uint32_t timestamp) {
    static uint32_t last_timestamp[GPIO_COUNT] = { 0 };
    static bool last_pin_state[GPIO_COUNT] = { false };
    static bool last_sample_valid[GPIO_COUNT] = { false };

    if (channel > sizeof(pwm_in_gpios) / sizeof(pwm_in_gpios[0]))
        return;
    int gpio_num = pwm_in_gpios[channel];
    if (gpio_num < 1 || gpio_num > GPIO_COUNT)
        return;
    bool current_pin_state = gpios[gpio_num - 1]->read();

    if (last_sample_valid[gpio_num - 1]
        && (last_pin_state[gpio_num - 1] != PWM_INVERT_INPUT)
        && (current_pin_state == PWM_INVERT_INPUT)) {
        handle_pulse(gpio_num, timestamp - last_timestamp[gpio_num - 1]);
    }

    last_timestamp[gpio_num - 1] = timestamp;
    last_pin_state[gpio_num - 1] = current_pin_state;
    last_sample_valid[gpio_num - 1] = true;
}

bool pwm_in_init(STM32_Timer_t* timer, uint8_t priority) {
    if (!timer->init(0xFFFFFFFF /* period */, STM32_Timer_t::UP))
        return false;
    
    STM32_GPIO_t* active_gpios[4] = { nullptr };
    for (size_t i = 0; i < 4; ++i) {
        uint32_t gpio_num = pwm_in_gpios[i];
        if (gpio_num && is_endpoint_ref_valid(board_config.pwm_mappings[gpio_num - 1].endpoint)) {
            active_gpios[i] = gpios[gpio_num - 1];
        }
    };

    if (!timer->config_input_capture_mode(active_gpios[0], active_gpios[1], active_gpios[2], active_gpios[3]))
        return false;
    if (!timer->on_cc_.set<void>(pwm_in_cb, nullptr))
        return false;
    if (!timer->enable_cc_interrupt(priority))
        return false;

/*
    TIM_IC_InitTypeDef sConfigIC;
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    for (int gpio_num = 1; gpio_num <= 4; ++gpio_num) {
#else
    int gpio_num = 4; {
#endif
        if (is_endpoint_ref_valid(board_config.pwm_mappings[gpio_num - 1].endpoint)) {
            // TODO: the owner should give up the GPIO first
            GPIO_InitStruct.Pin = 1U << gpios[gpio_num]->pin_number;
            HAL_GPIO_Deinit(gpios[gpio_num]->port);
            HAL_GPIO_Init(gpios[gpio_num]->port, &GPIO_InitStruct);

            HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, gpio_num_to_tim_2_5_channel(gpio_num));
            HAL_TIM_IC_Start_IT(&htim5, gpio_num_to_tim_2_5_channel(gpio_num));
        }
    }*/
    return true;
}
