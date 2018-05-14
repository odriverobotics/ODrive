#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#include "drv8301.h"

typedef enum {
    MOTOR_TYPE_HIGH_CURRENT = 0,
    // MOTOR_TYPE_LOW_CURRENT = 1, //Not yet implemented
    MOTOR_TYPE_GIMBAL = 2
} Motor_type_t;

typedef struct {
    float phB;
    float phC;
} Iph_BC_t;

typedef struct {
    float p_gain; // [V/A]
    float i_gain; // [V/As]
    float v_current_control_integral_d; // [V]
    float v_current_control_integral_q; // [V]
    float Ibus; // DC bus current [A]
    // Voltage applied at end of cycle:
    float final_v_alpha; // [V]
    float final_v_beta; // [V]
    float Iq_setpoint;
    float Iq_measured;
    float max_allowed_current;
} Current_control_t;

// NOTE: for gimbal motors, all units of A are instead V.
// example: vel_gain is [V/(count/s)] instead of [A/(count/s)]
// example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
typedef struct {
    bool pre_calibrated = false; // can be set to true to indicate that all values here are valid
    int32_t pole_pairs = 7;
    float calibration_current = 10.0f;    // [A]
    float resistance_calib_max_voltage = 1.0f; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
    float phase_inductance = 0.0f;        // to be set by measure_phase_inductance
    float phase_resistance = 0.0f;        // to be set by measure_phase_resistance
    int32_t direction = 1;                // 1 or -1
    Motor_type_t motor_type = MOTOR_TYPE_HIGH_CURRENT;

    // Read out max_allowed_current to see max supported value for current_lim.
    // float current_lim = 70.0f; //[A]
    float current_lim = 10.0f;  //[A]
    // Value used to compute shunt amplifier gains
    float requested_current_range = 70.0f; // [A]
} MotorConfig_t;

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
        ERROR_UNEXPECTED_TIMER_CALLBACK = 0x0200
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

    Motor(const MotorHardwareConfig_t& hw_config,
         const GateDriverHardwareConfig_t& gate_driver_config,
         MotorConfig_t& config);

    bool arm();
    void disarm();
    void setup() {
        update_current_controller_gains();
        DRV8301_setup();
    }
    void reset_current_control();

    void update_current_controller_gains();
    void DRV8301_setup();
    bool check_DRV_fault();
    void set_error(Error_t error);
    bool do_checks();
    void log_timing(TimingLog_t log_idx);
    float phase_current_from_adcval(uint32_t ADCValue);
    bool measure_phase_resistance(float test_current, float max_voltage);
    bool measure_phase_inductance(float voltage_low, float voltage_high);
    bool run_calibration();
    bool enqueue_modulation_timings(float mod_alpha, float mod_beta);
    bool enqueue_voltage_timings(float v_alpha, float v_beta);
    bool FOC_voltage(float v_d, float v_q, float phase);
    bool FOC_current(float Id_des, float Iq_des, float phase);
    bool update(float current_setpoint, float phase);

    const MotorHardwareConfig_t& hw_config_;
    const GateDriverHardwareConfig_t gate_driver_config_;
    MotorConfig_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

//private:

    DRV8301_Obj gate_driver_; // initialized in constructor
    uint16_t next_timings_[3] = {
        TIM_1_8_PERIOD_CLOCKS / 2,
        TIM_1_8_PERIOD_CLOCKS / 2,
        TIM_1_8_PERIOD_CLOCKS / 2
    };
    bool next_timings_valid_ = false;
    uint16_t last_cpu_time_ = 0;
    int timing_log_index_ = 0;
    uint16_t timing_log_[TIMING_LOG_NUM_SLOTS] = { 0 };

    // variables exposed on protocol
    Error_t error_ = ERROR_NONE;
    // Do not write to this variable directly!
    // It is for exclusive use by the safety_critical_... functions.
    ArmedState_t armed_state_ = ARMED_STATE_DISARMED; 
    bool is_calibrated_ = config_.pre_calibrated;
    Iph_BC_t current_meas_ = {0.0f, 0.0f};
    Iph_BC_t DC_calib_ = {0.0f, 0.0f};
    float phase_current_rev_gain_ = 0.0f; // Reverse gain for ADC to Amps (to be set by DRV8301_setup)
    Current_control_t current_control_ = {
        .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
        .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
        .v_current_control_integral_d = 0.0f,
        .v_current_control_integral_q = 0.0f,
        .Ibus = 0.0f,
        .final_v_alpha = 0.0f,
        .final_v_beta = 0.0f,
        .Iq_setpoint = 0.0f,
        .Iq_measured = 0.0f,
        .max_allowed_current = 0.0f,
    };
    DRV8301_FaultType_e drv_fault_ = DRV8301_FaultType_NoFault;
    DRV_SPI_8301_Vars_t gate_driver_regs_; //Local view of DRV registers (initialized by DRV8301_setup)

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
            make_protocol_property("phase_current_rev_gain", &phase_current_rev_gain_),
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
                make_protocol_property("max_allowed_current", &current_control_.max_allowed_current)
            ),
            make_protocol_object("gate_driver",
                make_protocol_ro_property("drv_fault", &drv_fault_)
                // make_protocol_ro_property("status_reg_1", &gate_driver_regs_.Stat_Reg_1_Value),
                // make_protocol_ro_property("status_reg_2", &gate_driver_regs_.Stat_Reg_2_Value),
                // make_protocol_ro_property("ctrl_reg_1", &gate_driver_regs_.Ctrl_Reg_1_Value),
                // make_protocol_ro_property("ctrl_reg_2", &gate_driver_regs_.Ctrl_Reg_2_Value)
            ),
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
                make_protocol_property("pole_pairs", &config_.pole_pairs),
                make_protocol_property("calibration_current", &config_.calibration_current),
                make_protocol_property("resistance_calib_max_voltage", &config_.resistance_calib_max_voltage),
                make_protocol_property("phase_inductance", &config_.phase_inductance),
                make_protocol_property("phase_resistance", &config_.phase_resistance),
                make_protocol_property("direction", &config_.direction),
                make_protocol_property("motor_type", &config_.motor_type),
                make_protocol_property("current_lim", &config_.current_lim),
                make_protocol_property("requested_current_range", &config_.requested_current_range)
            )
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Motor::Error_t)

#endif // __MOTOR_HPP
