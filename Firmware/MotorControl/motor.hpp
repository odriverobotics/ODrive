#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#include <devices.hpp>
#include <thermistor.hpp>

class Motor {
public:
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
        float p_gain; // [V/A]
        float i_gain; // [V/As]
        float v_current_control_integral_d; // [V]
        float v_current_control_integral_q; // [V]
        float Ibus; // DC bus current [A]
        // Voltage applied at end of cycle:
        float final_v_alpha; // [V]
        float final_v_beta; // [V]
        float Iq_setpoint; // [A]
        float Iq_measured; // [A]
        float Id_measured; // [A]
        float I_measured_report_filter_k;
        float max_allowed_current; // [A]
        float overcurrent_trip_level; // [A]
    };

    // NOTE: for gimbal motors, all units of A are instead V.
    // example: vel_gain is [V/(count/s)] instead of [A/(count/s)]
    // example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
    struct Config_t {
        bool pre_calibrated = false; // can be set to true to indicate that all values here are valid
        int32_t pole_pairs = 7;
        float calibration_current = 10.0f;    // [A]
        float resistance_calib_max_voltage = 2.0f; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
        float phase_inductance = 0.0f;        // to be set by measure_phase_inductance
        float phase_resistance = 0.0f;        // to be set by measure_phase_resistance
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

    enum ArmedState_t {
        ARMED_STATE_DISARMED,
        ARMED_STATE_WAITING_FOR_TIMINGS,
        ARMED_STATE_WAITING_FOR_UPDATE,
        ARMED_STATE_ARMED,
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
         Thermistor_t* inverter_thermistor,
         uint16_t period, uint16_t repetition_counter, uint16_t dead_time,
         Config_t& config);

    bool arm();
    void disarm();

    void handle_timer_update();
    void handle_current_sensor_update();

    bool init();
    bool start_updates();

    void reset_current_control();
    void update_current_controller_gains();
    bool check_DRV_fault();
    void set_error(Error_t error);
    bool do_checks();
    float get_inverter_temp();
    bool update_thermal_limits();
    float effective_current_lim();
    bool measure_phase_resistance(float test_current, float max_voltage);
    bool measure_phase_inductance(float voltage_low, float voltage_high);
    bool run_calibration();
    bool enqueue_modulation_timings(float mod_alpha, float mod_beta);
    bool enqueue_voltage_timings(float v_alpha, float v_beta);
    bool FOC_voltage(float v_d, float v_q, float pwm_phase);
    bool FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase);
    bool update(float current_setpoint, float phase, float phase_vel);

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
    Thermistor_t* inverter_thermistor_;

    uint16_t period_;
    uint16_t repetition_counter_;
    uint16_t dead_time_;

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

//private:


    uint16_t GPIO_port_samples[n_GPIO_samples];

    volatile uint8_t new_current_readings_ = 0; // bitfield (values 0...7) to indicate which of the current measurements are new. If ==7, the next current control iteration can happen. Reset by the current control iteration.
    volatile bool is_updating_pwm_timings_ = false; // true while the PWM timings are being updated
    volatile uint32_t last_pwm_update_timestamp_ = 0xffffffff; // set to the current timer value after the PWM timings are committed
    uint16_t pwm_control_deadline_; // set in start()

    // variables exposed on protocol
    Error_t error_ = ERROR_NONE;
    // Do not write to this variable directly!
    // It is for exclusive use by the safety_critical_... functions.
    ArmedState_t armed_state_ = ARMED_STATE_DISARMED; 
    bool is_calibrated_ = config_.pre_calibrated;
    Iph_ABC_t current_meas_ = {0.0f, 0.0f};
    Iph_ABC_t DC_calib_ = {0.0f, 0.0f};
    CurrentControl_t current_control_ = {
        .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
        .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
        .v_current_control_integral_d = 0.0f,
        .v_current_control_integral_q = 0.0f,
        .Ibus = 0.0f,
        .final_v_alpha = 0.0f,
        .final_v_beta = 0.0f,
        .Iq_setpoint = 0.0f,
        .Iq_measured = 0.0f,
        .Id_measured = 0.0f,
        .I_measured_report_filter_k = 1.0f,
        .max_allowed_current = 0.0f,
        .overcurrent_trip_level = 0.0f,
    };
    float thermal_current_lim_ = 10.0f;  //[A]

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_ro_property("armed_state", &armed_state_),
            make_protocol_ro_property("is_calibrated", &is_calibrated_),
            make_protocol_ro_property("current_meas_phB", &current_meas_.phB),
            make_protocol_ro_property("current_meas_phC", &current_meas_.phC),
            make_protocol_property("DC_calib_phB", &DC_calib_.phB),
            make_protocol_property("DC_calib_phC", &DC_calib_.phC),
            make_protocol_ro_property("thermal_current_lim", &thermal_current_lim_),
            make_protocol_function("get_inverter_temp", *this, &Motor::get_inverter_temp),
            make_protocol_object("current_control",
                make_protocol_property("p_gain", &current_control_.p_gain),
                make_protocol_property("i_gain", &current_control_.i_gain),
                make_protocol_property("v_current_control_integral_d", &current_control_.v_current_control_integral_d),
                make_protocol_property("v_current_control_integral_q", &current_control_.v_current_control_integral_q),
                make_protocol_property("Ibus", &current_control_.Ibus),
                make_protocol_property("final_v_alpha", &current_control_.final_v_alpha),
                make_protocol_property("final_v_beta", &current_control_.final_v_beta),
                make_protocol_property("Iq_setpoint", &current_control_.Iq_setpoint),
                make_protocol_property("Iq_measured", &current_control_.Iq_measured),
                make_protocol_property("Id_measured", &current_control_.Id_measured),
                make_protocol_property("I_measured_report_filter_k", &current_control_.I_measured_report_filter_k),
                make_protocol_ro_property("max_allowed_current", &current_control_.max_allowed_current),
                make_protocol_ro_property("overcurrent_trip_level", &current_control_.overcurrent_trip_level)
            ),

            //make_protocol_object("gate_driver_a", gate_driver_a->make_protocol_definitions()),
            //make_protocol_object("gate_driver_b", gate_driver_b->make_protocol_definitions()),
            //make_protocol_object("gate_driver_c", gate_driver_c->make_protocol_definitions()),
            //make_protocol_object("current_sensor_a", current_sensor_a.make_protocol_definitions()),
            //make_protocol_object("current_sensor_b", current_sensor_b.make_protocol_definitions()),
            //make_protocol_object("current_sensor_c", current_sensor_c.make_protocol_definitions()),

//            make_protocol_object("timing_log",
//                make_protocol_ro_property("TIMING_LOG_GENERAL", &timing_log_[TIMING_LOG_GENERAL]),
//                make_protocol_ro_property("TIMING_LOG_ADC_CB_I", &timing_log_[TIMING_LOG_ADC_CB_I]),
//                make_protocol_ro_property("TIMING_LOG_ADC_CB_DC", &timing_log_[TIMING_LOG_ADC_CB_DC]),
//                make_protocol_ro_property("TIMING_LOG_MEAS_R", &timing_log_[TIMING_LOG_MEAS_R]),
//                make_protocol_ro_property("TIMING_LOG_MEAS_L", &timing_log_[TIMING_LOG_MEAS_L]),
//                make_protocol_ro_property("TIMING_LOG_ENC_CALIB", &timing_log_[TIMING_LOG_ENC_CALIB]),
//                make_protocol_ro_property("TIMING_LOG_IDX_SEARCH", &timing_log_[TIMING_LOG_IDX_SEARCH]),
//                make_protocol_ro_property("TIMING_LOG_FOC_VOLTAGE", &timing_log_[TIMING_LOG_FOC_VOLTAGE]),
//                make_protocol_ro_property("TIMING_LOG_FOC_CURRENT", &timing_log_[TIMING_LOG_FOC_CURRENT])
//            ),
            make_protocol_object("config",
                make_protocol_property("pre_calibrated", &config_.pre_calibrated),
                make_protocol_property("pole_pairs", &config_.pole_pairs),
                make_protocol_property("calibration_current", &config_.calibration_current),
                make_protocol_property("resistance_calib_max_voltage", &config_.resistance_calib_max_voltage),
                make_protocol_property("phase_inductance", &config_.phase_inductance),
                make_protocol_property("phase_resistance", &config_.phase_resistance),
                make_protocol_property("direction", &config_.direction),
                make_protocol_property("motor_type", &config_.motor_type),
                make_protocol_property("current_lim", &config_.current_lim),
                make_protocol_property("inverter_temp_limit_lower", &config_.inverter_temp_limit_lower),
                make_protocol_property("inverter_temp_limit_upper", &config_.inverter_temp_limit_upper),
                make_protocol_property("requested_current_range", &config_.requested_current_range),
                make_protocol_property("current_control_bandwidth", &config_.current_control_bandwidth,
                    [](void* ctx) { static_cast<Motor*>(ctx)->update_current_controller_gains(); }, this)
            )
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Motor::Error_t)

#endif // __MOTOR_HPP
