#ifndef __CURRENT_SENSOR_HPP
#define __CURRENT_SENSOR_HPP

#include "subscriber.hpp"
#include "adc.hpp"

struct CurrentSensor_t {
    /**
     * @brief Initializes the current sensor hardware.
     * Must be callable multiple times.
     */
    virtual bool init(float requested_range) = 0;

    /**
     * @brief Enables the update event of this current sensor.
     * The on_update_ event should be configured first. The update trigger
     * source depends on the type of the current sensor and on the setup of the
     * underlying ADC channel.
     * Returns true if the event was enabled successfully or false otherwise.
     */
    virtual bool enable_updates() = 0;

    /**
     * @brief Tries to set the range of the current sensor to at least
     * {-requested_range, ..., requested_range}.
     * The actual range that is set may be smaller or larger than the requested
     * range. Use get_range() to get the actual range.
     * Returns true if the range was updated or false otherwise.
     */
    virtual bool set_range(float requested_range) = 0;

    /**
     * @brief Returns the maximum current that the sensor can currently measure.
     * The range can be assumed to be symmetric, i.e. {-max_range ... +max_range}.
     * Returns true on success or false otherwise.
     */
    virtual bool get_range(float* max_range) = 0;

    /**
     * @brief Returns true if there is a new value since reset_value() was
     * last called. The underlying ADC must be running, else this will never
     * become true.
     */
    virtual bool has_value() = 0;

    /**
     * @brief Resets the state of has_value().
     */
    virtual bool reset_value() = 0;

    /**
     * @brief Returns the most recent current measurement in Amps.
     */
    virtual bool get_current(float* current) = 0;

    Subscriber<> on_update_;
};

template<typename TADCChannel,
        TADCChannel* adc,
        typename TOpAmp,
        TOpAmp* opamp>
class LinearCurrentSensor_t : public CurrentSensor_t {
public:
    LinearCurrentSensor_t(float adc_to_i, float adc_midpoint) :
        adc_to_i_(adc_to_i), adc_midpoint_(adc_midpoint) {}

    bool init(float requested_range) final {
        if (!is_setup_) {
            bool success = not_null(adc) &&
                           adc->init() &&
                           (is_null(opamp) || opamp->init()) &&
                           set_range(requested_range);
            if (!success) {
                return false;
            }
            Subscriber<> on_update = adc->on_update_;
            if (!on_update.set<LinearCurrentSensor_t, &LinearCurrentSensor_t::handle_update>(*this)) {
                return false;
            }
            is_setup_ = true;
        }
        return true;
    }

    void get_unity_gain_range(float* min_current, float* max_current) {
        float adc_min = not_null(opamp) ? std::max(-opamp->get_max_output_swing(), -adc_midpoint_) : -adc_midpoint_;
        float adc_max = not_null(opamp) ? std::min(opamp->get_max_output_swing(), 1 - adc_midpoint_) : (1 - adc_midpoint_);
        *min_current = adc_min * adc_to_i_ * rev_gain_;
        *max_current = adc_max * adc_to_i_ * rev_gain_;
    }

    bool set_range(float requested_range) final {
        if (not_null(opamp)) {
            float min_unity_gain_current;
            float max_unity_gain_current;
            get_unity_gain_range(&min_unity_gain_current, &max_unity_gain_current);
            float unity_gain_current_range = std::min(-min_unity_gain_current, max_unity_gain_current);
            float requested_gain = unity_gain_current_range / requested_range; // [V/V]

            // set gain
            float actual_gain = opamp->set_gain(requested_gain);
            rev_gain_ = 1.0f / actual_gain;
        } else {
            // if no opamp is present we just ignore the request and leave the gain at 1.0
            rev_gain_ = 1.0f;
        }
        return true;
    }

    bool get_range(float* max_current) final {
        float min_unity_gain_current;
        float max_unity_gain_current;
        get_unity_gain_range(&min_unity_gain_current, &max_unity_gain_current);
        if (max_current) {
            *max_current = std::min(-min_unity_gain_current, max_unity_gain_current);
        }
        return true;
    }

    bool has_value() final {
        return not_null(adc) && adc->has_value();
    }

    bool reset_value() final {
        return not_null(adc) && adc->reset_value();
    }

    bool enable_updates() final {
        if (not_null(adc)) {
            return adc->enable_updates();
        } else {
            return false;
        }
    }

    bool get_current(float* current) final {
        float adc_val = 0;
        if (!adc->get_normalized(&adc_val)) {
            return false;
        }
        if (current) {
            *current = (adc_val - adc_midpoint_) * adc_to_i_ * rev_gain_;
        }
        return true;
    }

private:
    float adc_to_i_; // factor to get from normalized ADC value to unity gain current value [A]
    float adc_midpoint_;
    bool is_setup_ = false;
    bool has_value_ = false;
    float rev_gain_ = 1.0f;

    void handle_update() {
        // TODO: set new current
        has_value_ = true;
        on_update_.invoke();
    }
};


#endif // __CURRENT_SENSOR_HPP