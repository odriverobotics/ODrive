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


#endif // __DEVICES_HPP