#ifndef __MOTOR_HPP
#define __MOTOR_HPP

class Axis; // declared in axis.hpp
class Motor;

#include <board.h>

#include <autogen/interfaces.hpp>

enum TimingLog_t {
    TIMING_LOG_GENERAL,
    TIMING_LOG_ADC_CB_I,
    TIMING_LOG_ADC_CB_DC,
    TIMING_LOG_MEAS_R,
    TIMING_LOG_MEAS_L,
    TIMING_LOG_ENC_CALIB,
    TIMING_LOG_IDX_SEARCH,
    TIMING_LOG_FOC_VOLTAGE,
    TIMING_LOG_FOC_CURRENT,
    TIMING_LOG_SPI_START,
    TIMING_LOG_SAMPLE_NOW,
    TIMING_LOG_SPI_END,
    TIMING_LOG_NUM_SLOTS
};

class Motor : public ODriveIntf::MotorIntf {
public:
    struct Iph_BC_t {
        float phB;
        float phC;
    };

    struct CurrentControl_t{
        float p_gain; // [V/A]
        float i_gain; // [V/As]
        float v_current_control_integral_d; // [V]
        float v_current_control_integral_q; // [V]
        float Ibus; // DC bus current [A]
        // Voltage applied at end of cycle:
        float final_v_alpha; // [V]
        float final_v_beta; // [V]
        float Id_setpoint; // [A]
        float Iq_setpoint; // [A]
        float Iq_measured; // [A]
        float Id_measured; // [A]
        float I_measured_report_filter_k;
        float max_allowed_current; // [A]
        float overcurrent_trip_level; // [A]
        float acim_rotor_flux; // [A]
        float async_phase_vel; // [rad/s electrical]
        float async_phase_offset; // [rad electrical]
    };

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
        int32_t direction = 0;                // 1 or -1 (0 = unspecified)
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
        float acim_slip_velocity = 14.706f; // [rad/s electrical] = 1/rotor_tau
        float acim_gain_min_flux = 10; // [A]
        float acim_autoflux_min_Id = 10; // [A]
        bool acim_autoflux_enable = false;
        float acim_autoflux_attack_gain = 10.0f;
        float acim_autoflux_decay_gain = 1.0f;
        bool R_wL_FF_enable = false; // Enable feedforwards for R*I and w*L*I terms
        bool bEMF_FF_enable = false; // Enable feedforward for bEMF

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
         uint16_t control_deadline,
         float shunt_conductance,
         TGateDriver& gate_driver,
         TOpAmp& opamp);

    bool arm();
    void disarm();
    bool apply_config();
    bool setup();
    void reset_current_control();

    void update_current_controller_gains();
    void set_error(Error error);
    bool do_checks();
    float effective_current_lim();
    float max_available_torque();
    void log_timing(TimingLog_t log_idx);
    float phase_current_from_adcval(uint32_t ADCValue);
    bool measure_phase_resistance(float test_current, float max_voltage);
    bool measure_phase_inductance(float voltage_low, float voltage_high);
    bool run_calibration();
    bool enqueue_modulation_timings(float mod_alpha, float mod_beta);
    bool enqueue_voltage_timings(float v_alpha, float v_beta);
    bool FOC_voltage(float v_d, float v_q, float pwm_phase);
    bool FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase, float phase_vel);
    bool update(float current_setpoint, float phase, float phase_vel);
    void tim_update_cb();

    // hardware config
    TIM_HandleTypeDef* const timer_;
    const uint16_t control_deadline_;
    const float shunt_conductance_;
    TGateDriver& gate_driver_;
    TOpAmp& opamp_;

    Config_t config_;
    Axis* axis_ = nullptr; // set by Axis constructor

//private:

    uint16_t next_timings_[3] = {
        TIM_1_8_PERIOD_CLOCKS / 2,
        TIM_1_8_PERIOD_CLOCKS / 2,
        TIM_1_8_PERIOD_CLOCKS / 2
    };
    bool next_timings_valid_ = false;
    uint16_t last_cpu_time_ = 0;
    int timing_log_index_ = 0;
    struct {
        uint16_t& operator[](size_t idx) { return content[idx]; }
        uint16_t& get(size_t idx) { return content[idx]; }
        uint16_t content[TIMING_LOG_NUM_SLOTS];
    } timing_log_;

    // variables exposed on protocol
    Error error_ = ERROR_NONE;
    // Do not write to this variable directly!
    // It is for exclusive use by the safety_critical_... functions.
    ArmedState armed_state_ = ARMED_STATE_DISARMED; 
    bool is_calibrated_ = config_.pre_calibrated;
    Iph_BC_t current_meas_ = {0.0f, 0.0f};
    Iph_BC_t DC_calib_ = {0.0f, 0.0f};
    float phase_current_rev_gain_ = 0.0f; // Reverse gain for ADC to Amps (to be set by DRV8301_setup)
    CurrentControl_t current_control_ = {
        .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
        .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
        .v_current_control_integral_d = 0.0f,
        .v_current_control_integral_q = 0.0f,
        .Ibus = 0.0f,
        .final_v_alpha = 0.0f,
        .final_v_beta = 0.0f,
        .Id_setpoint = 0.0f,
        .Iq_setpoint = 0.0f,
        .Iq_measured = 0.0f,
        .Id_measured = 0.0f,
        .I_measured_report_filter_k = 1.0f,
        .max_allowed_current = 0.0f,
        .overcurrent_trip_level = 0.0f,
        .acim_rotor_flux = 0.0f,
        .async_phase_vel = 0.0f,
        .async_phase_offset = 0.0f,
    };
    float effective_current_lim_ = 10.0f; // [A]
};

#endif // __MOTOR_HPP
