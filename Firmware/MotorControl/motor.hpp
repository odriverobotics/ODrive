#ifndef __MOTOR_HPP
#define __MOTOR_HPP

class Axis; // declared in axis.hpp
class Motor;

#include <board.h>
#include <autogen/interfaces.hpp>
#include "foc.hpp"

class Motor : public ODriveIntf::MotorIntf {
public:

    // NOTE: for gimbal motors, all units of Nm are instead V.
    // example: vel_gain is [V/(turn/s)] instead of [Nm/(turn/s)]
    // example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
    struct Config_t {
        bool pre_calibrated = false; // can be set to true to indicate that all values here are valid
        int32_t pole_pairs = 7;
        float calibration_current = 10.0f;    // [A]
        float resistance_calib_max_voltage = 2.0f; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
        float phase_inductance = 0.0f;        // to be set by measure_phase_inductance
        float phase_resistance = 0.0f;        // to be set by measure_phase_resistance
        float torque_constant = 0.04f;         // [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
        MotorType motor_type = MOTOR_TYPE_HIGH_CURRENT;
        // Read out max_allowed_current to see max supported value for current_lim.
        // float current_lim = 70.0f; //[A]
        float current_lim = 10.0f;          //[A]
        float current_lim_margin = 8.0f;    // Maximum violation of current_lim
        float torque_lim = std::numeric_limits<float>::infinity();           //[Nm]. 
        // Value used to compute shunt amplifier gains
        float requested_current_range = 60.0f; // [A]
        float current_control_bandwidth = 1000.0f;  // [rad/s]
        float inverter_temp_limit_lower = 100;
        float inverter_temp_limit_upper = 120;

        float acim_gain_min_flux = 10; // [A]
        float acim_autoflux_min_Id = 10; // [A]
        bool acim_autoflux_enable = false;
        float acim_autoflux_attack_gain = 10.0f;
        float acim_autoflux_decay_gain = 1.0f;
        
        bool R_wL_FF_enable = false; // Enable feedforwards for R*I and w*L*I terms
        bool bEMF_FF_enable = false; // Enable feedforward for bEMF

        float I_bus_hard_min = -INFINITY;
        float I_bus_hard_max = INFINITY;
        float I_leak_max = 0.1f;

        float dc_calib_tau = 0.2f;

        // custom property setters
        Motor* parent = nullptr;
        void set_pre_calibrated(bool value) {
            pre_calibrated = value;
            parent->is_calibrated_ = parent->is_calibrated_ || parent->config_.pre_calibrated;
        }
        void set_phase_inductance(float value) { phase_inductance = value; parent->update_current_controller_gains(); }
        void set_phase_resistance(float value) { phase_resistance = value; parent->update_current_controller_gains(); }
        void set_current_control_bandwidth(float value) { current_control_bandwidth = value; parent->update_current_controller_gains(); }
    };

    Motor(TIM_HandleTypeDef* timer,
         uint8_t current_sensor_mask,
         float shunt_conductance,
         TGateDriver& gate_driver,
         TOpAmp& opamp,
         OnboardThermistorCurrentLimiter& fet_thermistor,
         OffboardThermistorCurrentLimiter& motor_thermistor);

    bool arm(PhaseControlLaw<3>* control_law);
    void apply_pwm_timings(uint16_t timings[3], bool tentative);
    bool disarm(bool* was_armed = nullptr);
    bool apply_config();
    bool setup();

    void update_current_controller_gains();
    void disarm_with_error(Error error);
    bool do_checks(uint32_t timestamp);
    float effective_current_lim();
    float max_available_torque();
    std::optional<float> phase_current_from_adcval(uint32_t ADCValue);
    bool measure_phase_resistance(float test_current, float max_voltage);
    bool measure_phase_inductance(float test_voltage);
    bool run_calibration();
    void update(uint32_t timestamp);

    // These functions are called as appropriate from the board.cpp file.
    void current_meas_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current);
    void dc_calib_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current);
    void pwm_update_cb(uint32_t output_timestamp);

    // hardware config
    TIM_HandleTypeDef* const timer_;
    const uint8_t current_sensor_mask_;
    const float shunt_conductance_;
    TGateDriver& gate_driver_;
    TOpAmp& opamp_;
    OnboardThermistorCurrentLimiter& fet_thermistor_;
    OffboardThermistorCurrentLimiter& motor_thermistor_;

    Config_t config_;
    Axis* axis_ = nullptr; // set by Axis constructor

//private:

    uint32_t n_evt_current_measurement_ = 0;
    uint32_t n_evt_pwm_update_ = 0;

    // variables exposed on protocol
    Error error_ = ERROR_NONE;
    float last_error_time_ = 0.0f;
    // Do not write to this variable directly!
    // It is for exclusive use by the safety_critical_... functions.
    bool is_armed_ = false;
    uint8_t armed_state_ = 0;
    bool is_calibrated_ = false; // Set in apply_config()
    std::optional<Iph_ABC_t> current_meas_;
    Iph_ABC_t DC_calib_ = {0.0f, 0.0f, 0.0f};
    float dc_calib_running_since_ = 0.0f; // current sensor calibration needs some time to settle
    float I_bus_ = 0.0f; // this motors contribution to the bus current
    float phase_current_rev_gain_ = 0.0f; // Reverse gain for ADC to Amps (to be set by DRV8301_setup)
    FieldOrientedController current_control_;
    float effective_current_lim_ = 10.0f; // [A]
    float max_allowed_current_ = 0.0f; // [A] set in setup()
    float max_dc_calib_ = 0.0f; // [A] set in setup()

    InputPort<float> torque_setpoint_src_; // Usually points to the Controller object's output
    InputPort<float> phase_vel_src_; // Usually points to the Encoder object's output

    float direction_ = 0.0f; // if -1 then positive torque is converted to negative Iq
    OutputPort<float2D> Vdq_setpoint_ = {{0.0f, 0.0f}}; // fed to the FOC
    OutputPort<float2D> Idq_setpoint_ = {{0.0f, 0.0f}}; // fed to the FOC
    
    PhaseControlLaw<3>* control_law_;
};


#endif // __MOTOR_HPP
