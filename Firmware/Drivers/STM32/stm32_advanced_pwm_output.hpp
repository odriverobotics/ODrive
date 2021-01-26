#ifndef __STM32_BASIC_PWM_OUTPUT_HPP
#define __STM32_BASIC_PWM_OUTPUT_HPP

#include <interfaces/pwm_output_group.hpp>

template<uint32_t CHANNEL>
volatile uint32_t& get_ccr(TIM_TypeDef* instance) {
    static_assert((CHANNEL == TIM_CHANNEL_1)
               || (CHANNEL == TIM_CHANNEL_2)
               || (CHANNEL == TIM_CHANNEL_3)
               || (CHANNEL == TIM_CHANNEL_4), "invalid channel");
    return (CHANNEL == TIM_CHANNEL_1) ? instance->CCR1 :
           (CHANNEL == TIM_CHANNEL_2) ? instance->CCR2 :
           (CHANNEL == TIM_CHANNEL_3) ? instance->CCR3 :
           instance->CCR4;
}

/**
 * @brief Implements the PwmOutputGroup<N> interface based on an STM32
 * advanced timer (TIM1 or TIM8).
 * 
 * This implementation handles safe enabling and disabling of MOE as well as
 * individual PWM channels.
 */
template<uint32_t PERIOD, uint32_t DEADTIME, uint32_t TIMER_BASE, uint32_t ... CHANNELS>
class Stm32AdvancedPwmOutput : public PwmOutputGroup<(sizeof...(CHANNELS))> {
public:
    using typename PwmOutputGroup<(sizeof...(CHANNELS))>::on_update_cb_t;
    using typename PwmOutputGroup<(sizeof...(CHANNELS))>::on_stopped_cb_t;

    void start(on_update_cb_t on_update, on_stopped_cb_t on_stopped) final {
        on_stopped_ = on_stopped;
        on_update_ = on_update;
    }

    /**
     * @brief Runs an update cycle. This should be called periodically
     * regardless of output state.
     * 
     * Must not be preempted by start() or stop() calls.
     */
    void update(timestamp_t timestamp) {
        std::array<uint32_t, sizeof...(CHANNELS)> ccr;
        ccr.fill(PERIOD / 2);

        uint32_t ccer_enable = 0x00000000;
        uint32_t ccer_disable = 0xffffffff;

        bool arm_or_stay_armed = false;
        bool disarm = false;

        if (on_update_) {
            auto result = on_update_.invoke(timestamp);

            if (!result.has_value()) {
                disarm = true;
            } else {
                for (size_t i = 0; i < sizeof...(CHANNELS); ++i) {
                    std::optional<float> duty_cycle_opt = result.value()[0];

                    if (duty_cycle_opt.has_value()) {
                        float duty_cycle = *duty_cycle_opt;

                        if (is_nan(duty_cycle) || (duty_cycle < 0.0f) || (duty_cycle > 1.0f)) {
                            // Invalid command by the application
                            disarm = true;
                        } else {
                            // This deadtime handling assumes center-aligned mode
                            // Lowest CCR value means maximum high-time
                            ccr[i] = (uint32_t)((PERIOD - DEADTIME) * (1.0f - duty_cycle)) + DEADTIME / 2;

                            // Enable this output channel
                            ccer_enable |= (TIM_CCx_ENABLE << channels_[i]);
                            ccer_enable |= (TIM_CCxN_ENABLE << channels_[i]);

                            // Request that the master output gets/stays enabled
                            arm_or_stay_armed = true;
                        }

                    } else {
                        // Disable this output channel
                        ccer_disable &= ~(TIM_CCx_ENABLE << channels_[i]);
                        ccer_disable &= ~(TIM_CCxN_ENABLE << channels_[i]);

                        // Request that the master output gets/stays enabled.
                        // This particular channel will remain disabled though.
                        arm_or_stay_armed = true;
                    }
                }
            }
        }

        // Write to CCR, CCER, and BDTR registers. All values written in this
        // section are latched on the next update event (except for MOE). If an
        // update event occurs during this section, the write operations may not
        // be atomic.
        CRITICAL_SECTION() {
            if (!arm_or_stay_armed) {
                // Prevent the PWMs from automatically enabling at the next update
                tim->BDTR &= ~TIM_BDTR_AOE;

                // Disable outputs immediately
                tim->BDTR &= ~TIM_BDTR_MOE;

            } else {
                // Disable all channels that did not get a duty cycle update
                uint32_t ccer = tim->CCER;
                ccer &= ccer_disable;
                tim->CCER = ccer;

                // Apply new duty cycle values
                for (size_t i = 0; i < sizeof...(CHANNELS); ++i) {
                    *(ccr_[i]) = ccr[i];
                }

                // Enable the channels that got a valid duty cycle update
                ccer |= ccer_enable;
                tim->CCER = ccer;

                // Set the Automatic Output Enable so that the Master Output Enable
                // bit will be automatically enabled on the next update event.
                tim->BDTR |= TIM_BDTR_AOE;
            }
        }

        if (disarm) {
            on_update_ = {};
            on_stopped_.invoke_and_clear();
        }
    }

    void stop() final {
        on_stopped_cb_t on_stopped;

        CRITICAL_SECTION() {
            // Prevent the PWMs from automatically enabling at the next update
            tim->BDTR &= ~TIM_BDTR_AOE;

            // Disable outputs immediately
            tim->BDTR &= ~TIM_BDTR_MOE;

            on_stopped = on_stopped_;
            on_stopped_ = {};
        }

        on_stopped.invoke();
    }

    TIM_TypeDef* const tim = (TIM_TypeDef*)TIMER_BASE;
    const std::array<uint32_t, sizeof...(CHANNELS)> channels_ = {CHANNELS...};
    const std::array<volatile uint32_t*, sizeof...(CHANNELS)> ccr_ = {
        &get_ccr<CHANNELS>(tim)...
    };

    on_update_cb_t on_update_;
    on_stopped_cb_t on_stopped_;
};

#endif // __STM32_BASIC_PWM_OUTPUT_HPP
