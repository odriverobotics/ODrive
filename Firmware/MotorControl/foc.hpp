#ifndef __FOC_HPP
#define __FOC_HPP

#include "phase_control_law.hpp"
#include "component.hpp"

/**
 * @brief Field oriented controller.
 * 
 * This controller can run in either current control mode or voltage control
 * mode.
 */
class FieldOrientedController : public AlphaBetaFrameController, public ComponentBase {
public:
    void update(uint32_t timestamp) final;

    void reset() final;
    
    ODriveIntf::MotorIntf::Error on_measurement(
            float vbus_voltage, float Ialpha, float Ibeta, uint32_t input_timestamp) final;

    ODriveIntf::MotorIntf::Error get_alpha_beta_output(
            uint32_t output_timestamp, float* mod_alpha, float* mod_beta, float* ibus) final;

    // Config - these values are set while this controller is inactive
    float p_gain_ = NAN; // [V/A] should be auto set after resistance and inductance measurement
    float i_gain_ = NAN; // [V/As] should be auto set after resistance and inductance measurement
    float I_measured_report_filter_k_ = 1.0f;

    // Inputs
    bool enable_current_control_src_ = false;
    float* Id_setpoint_src_ = nullptr;
    float* Iq_setpoint_src_ = nullptr;
    float* Vd_setpoint_src_ = nullptr;
    float* Vq_setpoint_src_ = nullptr;
    float* phase_src_ = nullptr;
    float* phase_vel_src_ = nullptr;

    // These values are set atomically by the update() function and read by the
    // calculate() function in an interrupt context.
    uint32_t ctrl_timestamp_; // [HCLK ticks]
    bool enable_current_control_ = false; // true: FOC runs in current control mode using I{dq}_setpoint, false: FOC runs in voltage control mode using V{dq}_setpoint
    float Id_setpoint_; // [A] only used if enable_current_control_ == true
    float Iq_setpoint_; // [A] only used if enable_current_control_ == true
    float Vd_setpoint_; // [V] acts as input if enable_current_control_ == false and as output otherwise
    float Vq_setpoint_; // [V] acts as input if enable_current_control_ == false and as output otherwise
    float phase_; // [rad]
    float phase_vel_; // [rad/s]

    // These values (or some of them) are updated inside on_measurement() and get_alpha_beta_output()
    uint32_t i_timestamp_;
    float vbus_voltage_measured_ = NAN; // [V]
    float Ialpha_measured_ = NAN; // [A]
    float Ibeta_measured_ = NAN; // [A]
    float Id_measured_ = 0.0f; // [A]
    float Iq_measured_ = 0.0f; // [A]
    float v_current_control_integral_d_ = 0.0f; // [V]
    float v_current_control_integral_q_ = 0.0f; // [V]
    //float mod_to_V_ = 0.0f;
    //float mod_d_ = 0.0f;
    //float mod_q_ = 0.0f;
    //float ibus_ = 0.0f;
    float final_v_alpha_ = 0.0f; // [V]
    float final_v_beta_ = 0.0f; // [V]
};

#endif // __FOC_HPP