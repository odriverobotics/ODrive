#ifndef __ADC_HPP
#define __ADC_HPP

#include "subscriber.hpp"

struct ADCChannel_t {
    /**
     * @brief Initializes the ADC hardware.
     * Must be callable multiple times.
     */
    virtual bool init() = 0;

    /**
     * @brief Returns the minimum and maximum voltages that this channel can
     * measure.
     * Returns true on success or false otherwise.
     */
    virtual bool get_range(float* min, float* max) = 0;

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
     * @brief Enables the update event on this ADC channel.
     * The underlying ADC must be configured beforehand, i.e. with a timer as
     * trigger or in continuous mode.
     */
    virtual bool enable_updates() = 0;

    Subscriber<> on_update_;
};


#endif // __ADC_HPP