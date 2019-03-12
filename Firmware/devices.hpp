#ifndef __DEVICES_HPP
#define __DEVICES_HPP

#include "subscriber.hpp"

struct ADCChannel_t {
    /**
     * @brief Initializes the ADC hardware.
     * Must be callable multiple times.
     */
    virtual bool init() = 0;

    /**
     * @brief Fetches the latest reading in volts.
     * Returns true if the value could be obtained or false otherwise.
     */
    virtual bool get_voltage(float *value) = 0;

    /**
     * @brief Fetches the latest reading as a number in {0...1}.
     * Returns true if the value could be obtained or false otherwise.
     */
    virtual bool get_normalized(float *value) = 0;

    /**
     * @brief Enables the update event on this ADC channel.
     * The underlying ADC must be configured beforehand, i.e. with a timer as
     * trigger or in continuous mode.
     */
    virtual bool enable_updates() = 0;

    Subscriber<> on_update_;
};

class VoltageDivider_t : public ADCChannel_t {
public:
    VoltageDivider_t(ADCChannel_t* adc, float divider_ratio) :
        adc_(adc),
        divider_ratio_(divider_ratio) {}

    bool init() final {
        return adc_
            && adc_->on_update_.set<VoltageDivider_t, &VoltageDivider_t::handle_update>(*this)
            && adc_->init();
    }

    bool get_voltage(float *value) final {
        if (adc_) {
            if (!adc_->get_voltage(value))
                return false;
            if (!value)
                return false;
            *value *= divider_ratio_;
            return true;
        } else {
            return false;
        }
    }

    bool get_normalized(float *value) final {
        return adc_ ? adc_->get_normalized(value) : false;
    }

    bool enable_updates() {
        if (!adc_) {
            return false;
        } else {
            return adc_->enable_updates();
            return true;
        }
    }

    ADCChannel_t* adc_;
    float divider_ratio_;
    Subscriber<> on_update_;

private:
    void handle_update() {
        on_update_.invoke();
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

    /** @brief Returns true if there is a new value since consume_value() was
     * last called. The underlying ADC must be running, else this will never
     * become true. */
    virtual bool has_value() = 0;

    /**
     * @brief Returns the most recent current measurement in Amps.
     */
    virtual bool get_current(float* current) = 0;

    /**
     * @brief Returns the most recent current measurement in Amps and resets the
     * state of has_value().
     */
    virtual bool consume_value(float* current) = 0;

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

class Shunt_t : public CurrentSensor_t {
public:
    Shunt_t(ADCChannel_t* adc, OpAmp_t* opamp_, float conductance) :
        adc_(adc), opamp_(opamp_), conductance_(conductance) {}

    bool init(float requested_range) final {
        if (!is_setup_) {
            if (!(adc_ && adc_->init()
                && opamp_ && opamp_->init()
                && set_range(requested_range)
                && adc_->on_update_.set<Shunt_t, &Shunt_t::handle_update>(*this))) {
                return false;
            }
            is_setup_ = true;
        }
        return true;
    }

    bool set_range(float requested_range) final {
        if (opamp_) {
            const float max_output_swing = opamp_->get_max_output_swing(); // [V] out of amplifier TODO: respect ADC max input range
            float max_unity_gain_current = max_output_swing * conductance_; // [A]
            float requested_gain = max_unity_gain_current / requested_range; // [V/V]

            // set gain
            float actual_gain = opamp_->set_gain(requested_gain);
            rev_gain_ = 1.0f / actual_gain;
            //float actual_max_current = max_unity_gain_current * rev_gain_;
            //return actual_max_current;
            return true;
        } else {
            return false;
        }
    }

    bool get_range(float* max_current) final {
        if (opamp_) {
            const float max_output_swing = opamp_->get_max_output_swing(); // [V] out of amplifier TODO: respect ADC max input range
            float max_unity_gain_current = max_output_swing * conductance_; // [A]
            if (max_current) {
                *max_current = max_unity_gain_current * rev_gain_;
            }
            return true;
        } else {
            return false;
        }
    }

    bool enable_updates() final {
        if (adc_) {
            return adc_->enable_updates();
        } else {
            return false;
        }
    }

    bool has_value() final {
        return has_value_;
    }

    bool get_current(float* current) final {
        float amp_out_volt = 0;
        if (!adc_->get_voltage(&amp_out_volt)) {
            return false;
        }
        amp_out_volt -= opamp_->get_midpoint();
        float shunt_volt = amp_out_volt * rev_gain_;
        if (current) {
            *current = shunt_volt * conductance_;
        }
        return true;
    }

    bool consume_value(float* current) final {
        if (adc_) {
            has_value_ = false;
            return get_current(current);
        } else {
            return false;
        }
    }

private:
    ADCChannel_t* adc_;
    OpAmp_t* opamp_;
    float conductance_;
    bool is_setup_ = false;
    bool has_value_ = false;
    float rev_gain_;

    void handle_update() {
        // TODO: set new current
        has_value_ = true;
        on_update_.invoke();
    }

    //// Communication protocol definitions
    //auto make_protocol_definitions() {
    //    return make_protocol_member_list(
    //        make_protocol_property("rev_gain", &rev_gain_)
    //    );
    //}
};


#endif // __DEVICES_HPP