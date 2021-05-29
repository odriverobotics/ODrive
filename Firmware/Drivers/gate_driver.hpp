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
     * @brief Returns false if the gate driver is in a state where the output
     * drive stages are disarmed or not properly configured (e.g. because they
     * are not initialized or there was a fault condition).
     */
    virtual bool is_ready() = 0;
};

struct OpAmpBase {
    /**
     * @brief Returns false if the opamp is in a state where it's not operating
     * with the latest configured gain (e.g. because it was not initialized or
     * there was a fault condition).
     */
    virtual bool is_ready() = 0;

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