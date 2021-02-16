#ifndef __STM32_BASIC_PWM_OUTPUT_HPP
#define __STM32_BASIC_PWM_OUTPUT_HPP

#include <interfaces/pwm_output_group.hpp>

template<uint32_t PERIOD, uint32_t DEADTIME>
class Stm32BasicPwmOutput : public PwmOutputGroup<1> {
public:
    Stm32BasicPwmOutput(volatile uint32_t& ccr_low, volatile uint32_t& ccr_high)
        : ccr_low_(ccr_low), ccr_high_(ccr_high) {}

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
        uint32_t low_off = 0; // Immediately turn off low side
        uint32_t high_on = PERIOD + 1; // Never turn on high side
        bool disarm = false;

        if (on_update_) {
            auto result = on_update_.invoke(timestamp);

            if (!result.has_value()) {
                disarm = true;
            } else {
                std::optional<float> duty_cycle_opt = result.value()[0];

                if (duty_cycle_opt.has_value()) {
                    float duty_cycle = *duty_cycle_opt;

                    if (is_nan(duty_cycle) || (duty_cycle < 0.0f) || (duty_cycle > 1.0f)) {
                        // Invalid command by the application
                        disarm = true;
                    } else {
                        high_on = (uint32_t)((PERIOD - DEADTIME) * (1.0f - duty_cycle)) + DEADTIME;
                        low_off = high_on - DEADTIME;
                    }

                } else {
                    // nothing to do (outputs get disabled by default)
                }
            }
        }

        CRITICAL_SECTION() {
            // Safe update of low and high side timings
            // To avoid race condition, first reset timings to safe state
            // ch3 is low side, ch4 is high side
            ccr_low_ = 0;
            ccr_high_ = PERIOD + 1;
            ccr_low_ = low_off;
            ccr_high_ = high_on;
        }

        if (disarm) {
            on_update_ = {};
            on_stopped_.invoke_and_clear();
        }
    }

    void stop() final {
        on_stopped_cb_t on_stopped;

        CRITICAL_SECTION() {
            ccr_low_ = 0;
            ccr_high_ = PERIOD + 1;

            on_stopped = on_stopped_;
            on_stopped_ = {};
        }

        on_stopped.invoke();
    }

    volatile uint32_t& ccr_low_;
    volatile uint32_t& ccr_high_;
    on_update_cb_t on_update_;
    on_stopped_cb_t on_stopped_;
};

#endif // __STM32_BASIC_PWM_OUTPUT_HPP
