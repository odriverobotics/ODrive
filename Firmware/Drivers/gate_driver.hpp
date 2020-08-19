#ifndef __GATE_DRIVER_HPP
#define __GATE_DRIVER_HPP

struct GateDriverBase {
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
     * fault state and true if it is in a nominal state.
     */
    virtual bool check_fault() = 0;
};

struct OpAmpBase {
    /**
     * @brief Tries to set the OpAmp gain to the specified value or lower.
     */
    virtual bool set_gain(float requested_gain, float* actual_gain) = 0;

    /**
     * @brief Returns the neutral voltage of the OpAmp in Volts
     */
    virtual float get_midpoint() = 0;

    /**
     * @brief Returns the maximum voltage swing away from the midpoint voltage (in Volts)
     */
    virtual float get_max_output_swing() = 0;
};

#endif // __GATE_DRIVER_HPP