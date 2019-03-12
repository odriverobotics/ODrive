

void pwm_in_init() {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;

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
    }
}



void pwm_in_cb(void* ctx, int channel, uint32_t timestamp) {
    static uint32_t last_timestamp[GPIO_COUNT] = { 0 };
    static bool last_pin_state[GPIO_COUNT] = { false };
    static bool last_sample_valid[GPIO_COUNT] = { false };

    int gpio_num = tim_2_5_channel_num_to_gpio_num(channel);
    if (gpio_num < 1 || gpio_num > GPIO_COUNT)
        return;
    bool current_pin_state = gpios[gpio_num]->read();

    if (last_sample_valid[gpio_num - 1]
        && (last_pin_state[gpio_num - 1] != PWM_INVERT_INPUT)
        && (current_pin_state == PWM_INVERT_INPUT)) {
        handle_pulse(gpio_num, timestamp - last_timestamp[gpio_num - 1]);
    }

    last_timestamp[gpio_num - 1] = timestamp;
    last_pin_state[gpio_num - 1] = current_pin_state;
    last_sample_valid[gpio_num - 1] = true;
}

//TODO: These expressions have integer division by 1MHz, so it will be incorrect for clock speeds of not-integer MHz
#define TIM_2_5_CLOCK_HZ        TIM_APB1_CLOCK_HZ
#define PWM_MIN_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 1000UL) // 1ms high is considered full reverse
#define PWM_MAX_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2000UL) // 2ms high is considered full forward
#define PWM_MIN_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 500UL) // ignore high periods shorter than 0.5ms
#define PWM_MAX_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2500UL) // ignore high periods longer than 2.5ms
#define PWM_INVERT_INPUT        false

void handle_pulse(int gpio_num, uint32_t high_time) {
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

