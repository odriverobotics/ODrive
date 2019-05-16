#ifndef __DEVICES_HPP
#define __DEVICES_HPP

#include "subscriber.hpp"
#include "adc.hpp"

#include <math.h>
#include <array>

class VoltageSensor_t {
public:
    virtual ~VoltageSensor_t() = default;

    virtual bool init() = 0;
    virtual bool get_range(float* min, float* max) = 0;
    virtual bool get_voltage(float *value) = 0;
};

class ADCVoltageSensor_t : public VoltageSensor_t {
public:
    ADCVoltageSensor_t(ADCChannel_t* adc, float full_scale) :
        adc_(adc),
        full_scale_(full_scale) {}

    bool init() final {
        return adc_ && adc_->init();
    }

    bool get_range(float* min, float* max) final {
        if (adc_) {
            if (min)
                *min = 0;
            if (max)
                *max = full_scale_;
            return true;
        } else {
            return false;
        }
    }

    bool get_voltage(float *value) final {
        if (adc_) {
            if (!adc_->get_normalized(value))
                return false;
            if (value)
                *value *= full_scale_;
            return true;
        } else {
            return false;
        }
    }

    ADCChannel_t* adc_;
    float full_scale_;
};

template<size_t N>
class AveragingVoltageSensor_t : public VoltageSensor_t {
public:
    std::array<VoltageSensor_t*, N> sensors_;
    template<typename ... Ts>
    AveragingVoltageSensor_t(Ts ... sensors) : sensors_{sensors...} {}
    
    bool init() final {
        for (size_t i = 0; i < N; ++i) {
            if (!sensors_[i] || !sensors_[i]->init()) {
                return false;
            }
        }
        return true;
    }
    bool get_range(float* min, float* max) final {
        float _min = INFINITY, _max = -INFINITY;
        for (size_t i = 0; i < N; ++i) {
            float temp_min, temp_max;
            if (!sensors_[i] || !sensors_[i]->get_range(&temp_min, &temp_max))
                return false;
            _min = std::min(_min, temp_min);
            _max = std::max(_max, temp_max);
        }
        if (min) *min = _min;
        if (max) *max = _max;
        return true;
    }
    bool get_voltage(float *value) final {
        float sum = 0.0f;
        for (size_t i = 0; i < N; ++i) {
            float val;
            if (!sensors_[i] || !sensors_[i]->get_voltage(&val))
                return false;
            sum += val;
        }
        if (value)
            *value = sum / (float)N;
        return true;
    }
};


struct SPI_t {
    /**
     * @brief Initializes the SPI peripheral.
     * Must be callable multiple times.
     */
    bool init();
};

struct GateDriver_t {
    /**
     * @brief Initializes the gate driver hardware.
     * Must be callable multiple times.
     */
    virtual bool init() = 0;

    /**
     * @brief Unlocks or locks the gate signals of the gate driver.
     * 
     * While locked the PWM inputs are ignored and the switches are always in
     * OFF state.
     * Not all gate drivers implement this function and may return true even if
     * the gate driver was not locked.
     */
    virtual bool set_enabled(bool enabled) = 0;

    /**
     * @brief Checks for a fault condition. Returns false if the driver is in a
     * fault state and true if it is in an nominal state.
     */
    virtual bool check_fault() = 0;

    /**
     * @brief Returns the error code of the gate driver.
     */
    virtual uint32_t get_error() = 0;
};


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

struct OpAmp_t {
    /**
     * @brief Initializes the opamp hardware.
     * Must be callable multiple times.
     */
    virtual bool init() = 0;

    /**
     * @brief Tries to set the OpAmp gain to the specified value or lower.
     * Returns the actual gain that was set.
     */
    virtual float set_gain(float max_gain) = 0;

    /**
     * @brief Returns the current gain setting
     */
    virtual float get_gain() = 0;

    /**
     * @brief Returns the neutral voltage of the OpAmp in Volts
     */
    virtual float get_midpoint() = 0;

    /**
     * @brief Returns the maximum voltage swing away from the midpoint voltage (in Volts)
     */
    virtual float get_max_output_swing() = 0;
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


#endif // __DEVICES_HPP