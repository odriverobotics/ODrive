#ifndef __STM32_TIM_HPP
#define __STM32_TIM_HPP

#include <stm32_gpio.hpp>
#include <subscriber.hpp>

class STM32_Timer_t {
public:
    enum MODE {
        UP,
        DOWN,
        UP_DOWN
    };

    TIM_HandleTypeDef htim;

    const STM32_GPIO_t** p_gpios[4];
    const STM32_GPIO_t** n_gpios[4];
    uint8_t gpio_af;

    /** @brief Triggered on a timer update interrupt */
    Subscriber<> on_update_;

    /** @brief Triggered on a timer trigger interrupt */
    Subscriber<> on_trigger_;

    /** @brief Triggered on a timer cc interrupt (can be either input or output compare) */
    Subscriber<uint32_t, uint32_t> on_cc_;

    /** @brief Triggered on a timer break interrupt */
    Subscriber<> on_break_;

    STM32_Timer_t(TIM_TypeDef* instance,
            const STM32_GPIO_t** ch1p_gpios, const STM32_GPIO_t** ch2p_gpios, const STM32_GPIO_t** ch3p_gpios, const STM32_GPIO_t** ch4p_gpios,
            const STM32_GPIO_t** ch1n_gpios, const STM32_GPIO_t** ch2n_gpios, const STM32_GPIO_t** ch3n_gpios, const STM32_GPIO_t** ch4n_gpios,
            uint8_t gpio_af) :
        htim { .Instance = instance },
        p_gpios{ch1p_gpios, ch2p_gpios, ch3p_gpios, ch4p_gpios},
        n_gpios{ch1n_gpios, ch2n_gpios, ch3n_gpios, ch4n_gpios},
        gpio_af(gpio_af)
    {}

    /**
     * @brief Initializes the timer.
     * This will generate an update event whether you want it or not, because... well... STM code.
     */
    bool init(uint32_t period, MODE mode, uint32_t prescaler = 0, uint32_t repetition_counter = 0);

    /**
     * @brief Configures PWM output on the specified channel.
     * The actual output must then be enabled using enable_pwm().
     * Call init() prior to calling this.
     * 
     * @param gpio_p: Specifies the pin to use for the positive output. NULL if
     *        the negative output channel is not used.
     *        Only a few pins are valid for each timer and channel. Refer to the
     *        datasheet for details
     * @param gpio_n: Specifies the pin to use for the negative output. NULL if
     *        the negative output channel is not used.
     *        Only a few pins are valid for each timer and channel. Refer to the
     *        datasheet for details
     * @param active_high_p: True if the positive output pin should be active high.
     * @param active_high_n: True if the negative output pin should be active high.
     */
    bool setup_pwm(uint32_t channel,
            STM32_GPIO_t* gpio_p, STM32_GPIO_t* gpio_n,
            bool active_high_p, bool active_high_n,
            uint32_t initial_val);

    /**
     * @brief Configures the dead-time generation for all PWM outputs of this
     * timer.
     */
    bool set_dead_time(uint32_t dead_time);

    bool config_encoder_mode(STM32_GPIO_t* gpio_ch1, STM32_GPIO_t* gpio_ch2);

    bool config_encoder_mode(GPIO_t* gpio_ch1, GPIO_t* gpio_ch2) {
        STM32_GPIO_t* stm32_gpio_ch1 = dynamic_cast<STM32_GPIO_t*>(gpio_ch1);
        STM32_GPIO_t* stm32_gpio_ch2 = dynamic_cast<STM32_GPIO_t*>(gpio_ch2);
        if ((gpio_ch1 && !stm32_gpio_ch1) || (gpio_ch2 && !stm32_gpio_ch2)) {
            return false;
        } else {
            return config_encoder_mode(stm32_gpio_ch1, stm32_gpio_ch2);
        }
    }

    /**
     * @brief Configures input compare on the specified channels
     */
    bool config_input_compare_mode(STM32_GPIO_t* gpio_ch3, STM32_GPIO_t* gpio_ch4);

    /**
     * @brief Sets the freeze-on-debug setting of the timer.
     * @param freeze_on_dbg: If true, the timer will freeze when the device is
     * halted by a debugger. For timers with complementary outputs (TIM1 and TIM8),
     * the outputs are also set to Hi-Z (see OCIdleState and OCNIdleState).
     * If false, the timer will continue running.
     */
    bool set_freeze_on_dbg(bool freeze_on_dbg);

    /**
     * @brief Enables PWM output on the specified outputs.
     * 
     * Call setup_pwm() for the corresponding channels first. Also call start()
     * first to make sure the timer is not stalled.
     * On advanced timers the outputs are enabled only once the Master Output
     * Enable bit is also set.
     */
    bool enable_pwm(bool ch1_p, bool ch1_n, bool ch2_p, bool ch2_n, bool ch3_p, bool ch3_n, bool ch4_p, bool ch4_n);

    bool start() {
        // TODO: there are separate "start" functions in the HAL for each mode
        return HAL_TIM_Base_Start_IT(&htim) == HAL_OK;
    }

    bool start_encoder() {
        return HAL_TIM_Encoder_Start(&htim, TIM_CHANNEL_ALL) == HAL_OK;
    }

    bool get_general_irqn(IRQn_Type* irqn);

    /** @brief Enables the update interrupt.
     * The on_update_ subscriber should be set first. */
    bool enable_update_interrupt();

    /** @brief Enables the trigger interrupt.
     * The on_trigger_ subscriber should be set first. */
    bool enable_trigger_interrupt();

    /** @brief Enables the cc interrupt.
     * The on_cc_ subscriber should be set first. */
    bool enable_cc_interrupt();

    /** @brief Enables the break interrupt.
     * The on_break_ subscriber should be set first. */
    bool enable_break_interrupt();

    void handle_update_irq();
    void handle_trigger_irq();
    void handle_cc_irq();
    void handle_break_irq();

    void handle_any_irq() {
        handle_update_irq();
        handle_trigger_irq();
        handle_cc_irq();
        handle_break_irq();
    }
};

void sync_timers(STM32_Timer_t* tim_a, STM32_Timer_t* tim_b,
                 uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset,
                 STM32_Timer_t* tim_refbase);


extern volatile uint32_t tim1_up_tim10_irq_ticks;
extern volatile uint32_t tim8_up_tim13_irq_ticks;

extern STM32_Timer_t tim1, tim2, tim3, tim4, tim5, tim6, tim7, tim8, tim9, tim10, tim11, tim12, tim13, tim13, tim14;

#endif // __STM32_TIM_HPP