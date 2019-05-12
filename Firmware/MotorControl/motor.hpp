#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#include <devices.hpp>
#include <thermistor.hpp>

class Motor {
public:
    typedef bool(*control_law_t)(Motor& motor, void* ctx, float pwm_timings[3]);

    enum Error_t {
        ERROR_NONE = 0,
        ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x0001,
        ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x0002,
        ERROR_ADC_FAILED = 0x0004,
        ERROR_DRV_FAULT = 0x0008,
        ERROR_CONTROL_DEADLINE_MISSED = 0x0010,
        ERROR_NOT_IMPLEMENTED_MOTOR_TYPE = 0x0020,
        ERROR_BRAKE_CURRENT_OUT_OF_RANGE = 0x0040,
        ERROR_MODULATION_MAGNITUDE = 0x0080,
        ERROR_BRAKE_DEADTIME_VIOLATION = 0x0100,
        ERROR_UNEXPECTED_TIMER_CALLBACK = 0x0200,
        ERROR_CURRENT_SENSE_SATURATION = 0x0400,
        ERROR_INVERTER_OVER_TEMP = 0x0800,
        ERROR_CURRENT_SENSOR = 0x1000,
        ERROR_BRAKE_RESISTOR_DISARMED = 0x2000,
        ERROR_NOT_CALIBRATED = 0x4000,                  // current control was used without calibrating phase R and L first
        ERROR_CURRENT_SENSOR_DEAD = 0x8000,
        ERROR_V_BUS_SENSOR_DEAD = 0x10000,
        ERROR_TOO_NOISY = 0x20000,
        ERROR_I_BUS_OUT_OF_RANGE = 0x40000,
        ERROR_TIMER_UPDATE_MISSED = 0x80000,
        ERROR_CONTROLLER_FAILED = 0x100000,
        ERROR_DC_BUS_UNDER_VOLTAGE = 0x200000,
        ERROR_DC_BUS_OVER_VOLTAGE = 0x400000,
        ERROR_FAILED_TO_ARM = 0x800000,
        ERROR_FOC_TIMEOUT = 0x1000000,
        ERROR_LEAK_CURRENT_TOO_HIGH = 0x2000000,
    };

    enum MotorType_t {
        MOTOR_TYPE_HIGH_CURRENT = 0,
        // MOTOR_TYPE_LOW_CURRENT = 1, //Not yet implemented
        MOTOR_TYPE_GIMBAL = 2
    };

    struct Iph_ABC_t {
        float phA;
        float phB;
        float phC;
    };

    struct CurrentControl_t{
        float p_gain = 0.0f; // [V/A] should be auto set after resistance and inductance measurement
        float i_gain = 0.0f; // [V/As] should be auto set after resistance and inductance measurement

        bool enable_current_control = false; // true: FOC runs in current control mode using I{dq}_setpoint, false: FOC runs in voltage control mode using V{dq}_setpoint
        float phase = 0.0f; // electrical phase of last current measurement [rad]
        float phase_vel = 0.0f; // electrical phase velocity [rad/s]
        float Id_setpoint = 0.0f; // [A]
        float Iq_setpoint = 0.0f; // [A]
        float Vd_setpoint = 0.0f; // [A]
        float Vq_setpoint = 0.0f; // [A]
        uint32_t cmd_expiry = 0; // time at which the FOC command expires

        float v_current_control_integral_d = 0.0f; // [V]
        float v_current_control_integral_q = 0.0f; // [V]

        // Voltage applied at end of cycle:
        float final_v_d = 0.0f; // [V]
        float final_v_q = 0.0f; // [V]
        float final_v_alpha = 0.0f; // [V]
        float final_v_beta = 0.0f; // [V]

        float Ibus = 0.0f; // DC bus current [A]

        float Iq_measured = 0.0f; // [A]
        float Id_measured = 0.0f; // [A]

        float I_measured_report_filter_k = 1.0f;
        float max_allowed_current = 0.0f; // [A]
        Iph_ABC_t overcurrent_trip_level = { 0.0f, 0.0f, 0.0f }; // [A]
    };

    // NOTE: for gimbal motors, all units of A are instead V.
    // example: vel_gain is [V/(count/s)] instead of [A/(count/s)]
    // example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
    struct Config_t {
        bool pre_calibrated = false; // if true, phase_inductance and phase_resistance are assumed to be valid
        bool async_calibrated = false; // if true, rotor_inductance, rotor_resistance and mutual_inductance are assumed to be valid

        int32_t pole_pairs = 7; // for linear motors put polepairs per meter here
        float calibration_current = 10.0f;    // [A]
        float resistance_calib_max_voltage = 2.0f; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.

        float phase_inductance = 0.0f;        // to be set by measure_phase_inductance
        float phase_resistance = 0.0f;        // to be set by measure_phase_resistance

        float rotor_inductance = 0.0f;      // [H] - only needed for induction motors
        float rotor_resistance = 0.0f;      // [Ohm] - only needed for induction motors
        float mutual_inductance = 0.0f;     // [H] - only needed for induction motors. Must be smaller than phase_inductance and rotor_inductance

        int32_t direction = 0;                // 1 or -1 (0 = unspecified)
        MotorType_t motor_type = MOTOR_TYPE_HIGH_CURRENT;
        // Read out max_allowed_current to see max supported value for current_lim.
        // float current_lim = 70.0f; //[A]
        float current_lim = 10.0f;  //[A]
        // Value used to compute shunt amplifier gains
        float requested_current_range = 60.0f; // [A]
        float current_control_bandwidth = 1000.0f;  // [rad/s]
        float inverter_temp_limit_lower = 100;
        float inverter_temp_limit_upper = 120;
        
        float phase_delay = 0.0f; // this is useful mostly if phase_locked is true. Must not be changed after calibrating the encoder with a synchronous motor

        float I_bus_hard_min = -INFINITY; // hard lower limit for bus current contribution
        float I_bus_hard_max = INFINITY; // hard upper limit for bus current contribution

        float max_leak_current = INFINITY; // [A] if three current sensors are available, the motor will disarm if this much current leaks out of the three phases

        float vbus_voltage_override = 0.0f; // if non-zero, overrides the DC voltage sensor (MAINLY INTENDED FOR DEVELOPMENT, USE WITH CAUTION!)

        float inv_temp_tau = 0.01f;
        float vbus_voltage_tau = 0.01f;
    };

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
        TIMING_LOG_NUM_SLOTS
    };

    enum UpdateMode_t {
        NONE = 0,
        ON_BOTTOM = 0x1,    // corresponds to SVM vector 7 (0b111)
        ON_TOP = 0x2,       // corresponds to SVM vector 0 (0b000)
        ON_BOTH = 0x3,
    };

    Motor(STM32_Timer_t* timer,
         STM32_GPIO_t* pwm_al_gpio, STM32_GPIO_t* pwm_bl_gpio, STM32_GPIO_t* pwm_cl_gpio,
         STM32_GPIO_t* pwm_ah_gpio, STM32_GPIO_t* pwm_bh_gpio, STM32_GPIO_t* pwm_ch_gpio,
         GateDriver_t* gate_driver_a,
         GateDriver_t* gate_driver_b,
         GateDriver_t* gate_driver_c,
         CurrentSensor_t* current_sensor_a,
         CurrentSensor_t* current_sensor_b,
         CurrentSensor_t* current_sensor_c,
         Thermistor_t* inverter_thermistor_a,
         Thermistor_t* inverter_thermistor_b,
         Thermistor_t* inverter_thermistor_c,
         VoltageSensor_t* vbus_sense,
         uint16_t period, uint16_t repetition_counter, uint16_t dead_time,
         uint8_t interrupt_priority,
         Config_t& config);

    bool arm(control_law_t control_law, void* ctx);
    bool arm_foc() {
        return arm(&Motor::FOC, &this->current_control_);
    }
    void disarm();

    void handle_timer_update();

    bool init();
    bool start_updates();

    void reset_current_control();
    void update_current_controller_gains();
    bool check_DRV_fault();
    void set_error(Error_t error);
    bool do_checks();
    bool update_thermal_limits();
    float effective_current_lim();
    void log_timing(TimingLog_t log_idx);
    bool pwm_test(float duration);
    bool measure_phase_resistance(float test_current, float max_voltage);
    bool measure_phase_inductance(float voltage_low, float voltage_high);
    bool run_calibration();
    static bool FOC(Motor& motor, void* ctx, float pwm_timings[3]);
    bool FOC_update(float Id_setpoint, float Iq_setpoint, float phase, float phase_vel, uint32_t expiry_us = 2000, bool force_voltage_control = false);

    STM32_Timer_t* timer_;
    STM32_GPIO_t* pwm_al_gpio_;
    STM32_GPIO_t* pwm_bl_gpio_;
    STM32_GPIO_t* pwm_cl_gpio_;
    STM32_GPIO_t* pwm_ah_gpio_;
    STM32_GPIO_t* pwm_bh_gpio_;
    STM32_GPIO_t* pwm_ch_gpio_;
    GateDriver_t* gate_driver_a_;
    GateDriver_t* gate_driver_b_;
    GateDriver_t* gate_driver_c_;
    CurrentSensor_t* current_sensor_a_;
    CurrentSensor_t* current_sensor_b_;
    CurrentSensor_t* current_sensor_c_;
    Thermistor_t* inverter_thermistor_a_;
    Thermistor_t* inverter_thermistor_b_;
    Thermistor_t* inverter_thermistor_c_;
    VoltageSensor_t* vbus_sense_;

    uint16_t period_;
    uint16_t repetition_counter_;
    uint16_t dead_time_;
    uint8_t interrupt_priority_;

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    UpdateMode_t pwm_update_mode_ = ON_BOTTOM;
    UpdateMode_t current_sample_mode_ = ON_BOTTOM;
    UpdateMode_t current_dc_calib_mode_ = ON_TOP;

//private:


    uint16_t GPIO_port_samples[n_GPIO_samples];

    volatile uint8_t new_current_readings_ = 0; // bitfield (values 0...7) to indicate which of the current measurements are new. If ==7, the next current control iteration can happen. Reset by the current control iteration.
    volatile bool is_updating_pwm_timings_ = false; // true while the PWM timings are being updated
    volatile bool did_refresh_pwm_timings_ = false; // set after new PWM timings are loaded into the timer, checked and reset after every timer update event by the ISR
    uint16_t timing_log_[TIMING_LOG_NUM_SLOTS] = { 0 };

    // variables exposed on protocol
    Error_t error_ = ERROR_NONE;
    // Do not write to this variable directly!
    // It is for exclusive use by the safety_critical_... functions.
    bool is_armed_ = false;
    bool is_calibrated_ = config_.pre_calibrated;
    Iph_ABC_t current_meas_ = {0.0f, 0.0f, 0.0f};
    Iph_ABC_t DC_calib_ = {0.0f, 0.0f, 0.0f};
    float I_alpha_beta_measured_[2] = {0.0f, 0.0f};
    float I_leak = 0.0f; // close to zero if only two current sensors are available
    bool current_sense_saturation_ = false; // if true, the measured current values must not be used for control
    float I_bus_ = 0.0f;

    float vbus_voltage_ = 1000.0f; // Arbitrary non-zero inital value to avoid division by zero if ADC reading is late

    uint32_t update_events_ = 0; // for debugging
    bool counting_down_ = false; // set on timer update event. First timer update event must be on upper peak.


    uint64_t longest_wait_ = 0;
    uint64_t max_it_ = 0;

    uint8_t field_weakening_status_ = 0;

    CurrentControl_t current_control_;
    float thermal_current_lim_ = 10.0f;  //[A]

    float inv_temp_a_ = -INFINITY;
    float inv_temp_b_ = -INFINITY;
    float inv_temp_c_ = -INFINITY;
    float max_inv_temp_ = -INFINITY;

    control_law_t control_law_ = nullptr; // set by arm() and reset by disarm()
    void* control_law_ctx_ = nullptr; // set by arm() and reset by disarm()

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_ro_property("is_armed", &is_armed_),
            make_protocol_ro_property("is_calibrated", &is_calibrated_),
            make_protocol_ro_property("vbus_voltage", &vbus_voltage_),
            make_protocol_ro_property("current_meas_phA", &current_meas_.phA),
            make_protocol_ro_property("current_meas_phB", &current_meas_.phB),
            make_protocol_ro_property("current_meas_phC", &current_meas_.phC),
            make_protocol_property("DC_calib_phA", &DC_calib_.phA),
            make_protocol_property("DC_calib_phB", &DC_calib_.phB),
            make_protocol_property("DC_calib_phC", &DC_calib_.phC),
            make_protocol_ro_property("I_alpha", &I_alpha_beta_measured_[0]),
            make_protocol_ro_property("I_beta", &I_alpha_beta_measured_[1]),
            make_protocol_ro_property("I_leak", &I_leak),
            make_protocol_ro_property("thermal_current_lim", &thermal_current_lim_),
            make_protocol_ro_property("inv_temp_a", &inv_temp_a_),
            make_protocol_ro_property("inv_temp_b", &inv_temp_b_),
            make_protocol_ro_property("inv_temp_c", &inv_temp_c_),
            make_protocol_property("max_inv_temp", &max_inv_temp_),
            make_protocol_ro_property("update_events", &update_events_),
            make_protocol_property("longest_wait", &longest_wait_),
            make_protocol_property("max_it", &max_it_),
            make_protocol_property("field_weakening_status", &field_weakening_status_),
            make_protocol_object("current_control",
                make_protocol_property("p_gain", &current_control_.p_gain),
                make_protocol_property("i_gain", &current_control_.i_gain),
                make_protocol_property("v_current_control_integral_d", &current_control_.v_current_control_integral_d),
                make_protocol_property("v_current_control_integral_q", &current_control_.v_current_control_integral_q),
                make_protocol_property("phase", &current_control_.phase),
                make_protocol_property("phase_vel", &current_control_.phase_vel),
                make_protocol_property("final_v_d", &current_control_.final_v_d),
                make_protocol_property("final_v_q", &current_control_.final_v_q),
                make_protocol_property("final_v_alpha", &current_control_.final_v_alpha),
                make_protocol_property("final_v_beta", &current_control_.final_v_beta),
                make_protocol_property("Id_setpoint", &current_control_.Id_setpoint),
                make_protocol_property("Iq_setpoint", &current_control_.Iq_setpoint),
                make_protocol_property("Vd_setpoint", &current_control_.Vd_setpoint),
                make_protocol_property("Vq_setpoint", &current_control_.Vq_setpoint),
                make_protocol_property("Id_measured", &current_control_.Id_measured),
                make_protocol_property("Iq_measured", &current_control_.Iq_measured),
                make_protocol_property("I_measured_report_filter_k", &current_control_.I_measured_report_filter_k),
                make_protocol_ro_property("max_allowed_current", &current_control_.max_allowed_current),
                make_protocol_ro_property("overcurrent_trip_level_a", &current_control_.overcurrent_trip_level.phA),
                make_protocol_ro_property("overcurrent_trip_level_b", &current_control_.overcurrent_trip_level.phB),
                make_protocol_ro_property("overcurrent_trip_level_c", &current_control_.overcurrent_trip_level.phC)
            ),


            //make_protocol_object("gate_driver_a", gate_driver_a->make_protocol_definitions()),
            //make_protocol_object("gate_driver_b", gate_driver_b->make_protocol_definitions()),
            //make_protocol_object("gate_driver_c", gate_driver_c->make_protocol_definitions()),

            make_protocol_function("gate_driver_c_get_fault", *gate_driver_c_,
                    &GateDriver_t::get_error),

            //make_protocol_object("gate_driver_a", gate_driver_a->make_protocol_definitions()),
            //make_protocol_object("gate_driver_b", gate_driver_b->make_protocol_definitions()),
            //make_protocol_object("gate_driver_c", gate_driver_c->make_protocol_definitions()),
            //make_protocol_object("current_sensor_a", current_sensor_a.make_protocol_definitions()),
            //make_protocol_object("current_sensor_b", current_sensor_b.make_protocol_definitions()),
            //make_protocol_object("current_sensor_c", current_sensor_c.make_protocol_definitions()),

            make_protocol_object("timing_log",
                make_protocol_ro_property("TIMING_LOG_GENERAL", &timing_log_[TIMING_LOG_GENERAL]),
                make_protocol_ro_property("TIMING_LOG_ADC_CB_I", &timing_log_[TIMING_LOG_ADC_CB_I]),
                make_protocol_ro_property("TIMING_LOG_ADC_CB_DC", &timing_log_[TIMING_LOG_ADC_CB_DC]),
                make_protocol_ro_property("TIMING_LOG_MEAS_R", &timing_log_[TIMING_LOG_MEAS_R]),
                make_protocol_ro_property("TIMING_LOG_MEAS_L", &timing_log_[TIMING_LOG_MEAS_L]),
                make_protocol_ro_property("TIMING_LOG_ENC_CALIB", &timing_log_[TIMING_LOG_ENC_CALIB]),
                make_protocol_ro_property("TIMING_LOG_IDX_SEARCH", &timing_log_[TIMING_LOG_IDX_SEARCH]),
                make_protocol_ro_property("TIMING_LOG_FOC_VOLTAGE", &timing_log_[TIMING_LOG_FOC_VOLTAGE]),
                make_protocol_ro_property("TIMING_LOG_FOC_CURRENT", &timing_log_[TIMING_LOG_FOC_CURRENT])
            ),
            make_protocol_object("config",
                make_protocol_property("pre_calibrated", &config_.pre_calibrated),
                make_protocol_property("async_calibrated", &config_.async_calibrated),
                make_protocol_property("pole_pairs", &config_.pole_pairs),
                make_protocol_property("calibration_current", &config_.calibration_current),
                make_protocol_property("resistance_calib_max_voltage", &config_.resistance_calib_max_voltage),
                make_protocol_property("phase_inductance", &config_.phase_inductance),
                make_protocol_property("phase_resistance", &config_.phase_resistance),
                make_protocol_property("rotor_inductance", &config_.rotor_inductance),
                make_protocol_property("rotor_resistance", &config_.rotor_resistance),
                make_protocol_property("mutual_inductance", &config_.mutual_inductance),
                make_protocol_property("direction", &config_.direction),
                make_protocol_property("motor_type", &config_.motor_type),
                make_protocol_property("current_lim", &config_.current_lim),
                make_protocol_property("inverter_temp_limit_lower", &config_.inverter_temp_limit_lower),
                make_protocol_property("inverter_temp_limit_upper", &config_.inverter_temp_limit_upper),
                make_protocol_property("requested_current_range", &config_.requested_current_range),
                make_protocol_property("current_control_bandwidth", &config_.current_control_bandwidth,
                    [](void* ctx) { static_cast<Motor*>(ctx)->update_current_controller_gains(); }, this),
                make_protocol_property("phase_delay", &config_.phase_delay),
                make_protocol_property("I_bus_hard_min", &config_.I_bus_hard_min),
                make_protocol_property("I_bus_hard_max", &config_.I_bus_hard_max),
                make_protocol_property("max_leak_current", &config_.max_leak_current),
                make_protocol_property("vbus_voltage_override", &config_.vbus_voltage_override),
                make_protocol_property("inv_temp_tau", &config_.inv_temp_tau)
            )
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Motor::Error_t)

#endif // __MOTOR_HPP
