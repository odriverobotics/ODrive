#ifndef __PHASE_CONTROL_LAW_HPP
#define __PHASE_CONTROL_LAW_HPP

#include <autogen/interfaces.hpp>
#include <variant>

template<size_t N_PHASES>
class PhaseControlLaw {
public:
    /**
     * @brief Called when this controller becomes the active controller.
     */
    virtual void reset() = 0;

    /**
     * @brief Informs the control law about a new set of measurements.
     *
     * This function gets called in a high priority interrupt context and should
     * run fast.
     *
     * Beware that all inputs can be NAN.
     *
     * @param vbus_voltage: The most recently measured DC link voltage. Can be
     *        std::nullopt if the measurement is not available or valid for any
     *        reason.
     * @param currents: The most recently measured (or inferred) phase currents
     *        in Amps. Can be std::nullopt if no valid measurements are available
     *        (e.g. because the opamp isn't started or because the sensors were
     *        saturated).
     * @param input_timestamp: The timestamp (in HCLK ticks) corresponding to
     *        the vbus_voltage and current measurement.
     */
    virtual ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<std::array<float, N_PHASES>> currents,
            uint32_t input_timestamp) = 0;

    /**
     * @brief Shall calculate the PWM timings for the specified target time.
     *
     * This function gets called in a high priority interrupt context and should
     * run fast.
     *
     * Beware that this function can be called before a call to on_measurement().
     * 
     * @param output_timestamp: The timestamp (in HCLK ticks) corresponding to
     *        the middle of the time span during which the output will be
     *        active.
     * @param pwm_timings: This array referenced by this argument shall be
     *        filled with the desired PWM timings. Each item corresponds to one
     *        phase and must lie in [0.0f, 1.0f].
     *        The function is not required to return valid PWM timings in case
     *        of an error.
     * @param ibus: The variable pointed to by this argument is set to the
     *        estimated DC current around the output timestamp when the desired
     *        PWM timings get applied.
     *        The function is not required to return a valid I_bus estimate in
     *        case of an error.
     * 
     * @returns: An error code or ERROR_NONE. If the function returns an error
     *           the motor gets disarmed with one exception: If the controller
     *           never returned valid PWM timings since it became active then it
     *           is allowed to return ERROR_CONTROLLER_INITIALIZING without
     *           triggering a motor disarm. In this phase the PWMs will not yet
     *           be truly active.
     */
    virtual ODriveIntf::MotorIntf::Error get_output(
            uint32_t output_timestamp,
            float (&pwm_timings)[N_PHASES],
            std::optional<float>* ibus) = 0;
};

class AlphaBetaFrameController : public PhaseControlLaw<3> {
private:
    ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<std::array<float, 3>> currents,
            uint32_t input_timestamp) final;

    ODriveIntf::MotorIntf::Error get_output(
            uint32_t output_timestamp,
            float (&pwm_timings)[3],
            std::optional<float>* ibus) final;

protected:
    virtual ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<float2D> Ialpha_beta,
            uint32_t input_timestamp) = 0;

    virtual ODriveIntf::MotorIntf::Error get_alpha_beta_output(
            uint32_t output_timestamp,
            std::optional<float2D>* mod_alpha_beta,
            std::optional<float>* ibus) = 0;
};

#endif // __PHASE_CONTROL_LAW_HPP