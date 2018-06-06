#ifndef __AXIS_HPP
#define __AXIS_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

// Warning: Do not reorder these enum values.
// The state machine uses ">" comparision on them.
enum AxisState_t {
    AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
    AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
    AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
    AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
    AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
};

struct AxisConfig_t {
    bool startup_motor_calibration = false;   //<! run motor calibration at startup, skip otherwise
    bool startup_encoder_index_search = false; //<! run encoder index search after startup, skip otherwise
                                               // this only has an effect if encoder.config.use_index is also true
    bool startup_encoder_offset_calibration = false; //<! run encoder offset calibration after startup, skip otherwise
    bool startup_closed_loop_control = false; //<! enable closed loop control after calibration/startup
    bool startup_sensorless_control = false; //<! enable sensorless control after calibration/startup
    bool enable_step_dir = false; //<! enable step/dir input after calibration
                                 //   For M0 this has no effect if enable_uart is true

    float counts_per_step = 2.0f;

    // Spinup settings
    float ramp_up_time = 0.4f;            // [s]
    float ramp_up_distance = 4 * M_PI;    // [rad]
    float spin_up_current = 10.0f;        // [A]
    float spin_up_acceleration = 400.0f;  // [rad/s^2]
    float spin_up_target_vel = 400.0f;    // [rad/s]
};

class Axis {
public:
    enum Error_t {
        ERROR_NONE = 0x00,
        ERROR_INVALID_STATE = 0x01, //<! an invalid state was requested
        ERROR_DC_BUS_UNDER_VOLTAGE = 0x02,
        ERROR_DC_BUS_OVER_VOLTAGE = 0x04,
        ERROR_CURRENT_MEASUREMENT_TIMEOUT = 0x08,
        ERROR_BRAKE_RESISTOR_DISARMED = 0x10, //<! the brake resistor was unexpectedly disarmed
        ERROR_MOTOR_DISARMED = 0x20, //<! the motor was unexpectedly disarmed
        ERROR_MOTOR_FAILED = 0x40,
        ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x80,
        ERROR_ENCODER_FAILED = 0x100,
        ERROR_CONTROLLER_FAILED = 0x200,
        ERROR_POS_CTRL_DURING_SENSORLESS = 0x400,
    };

    enum thread_signals {
        M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
    };

    Axis(const AxisHardwareConfig_t& hw_config,
            AxisConfig_t& config,
            Encoder& encoder,
            SensorlessEstimator& sensorless_estimator,
            Controller& controller,
            Motor& motor);

    void setup();
    void start_thread();
    void signal_current_meas();
    bool wait_for_current_meas();

    void step_cb();
    void set_step_dir_enabled(bool enable);

    bool check_DRV_fault();
    bool check_PSU_brownout();
    bool do_checks();
    bool do_updates();

    // @brief Runs the specified update handler at the frequency of the current measurements.
    //
    // The loop runs until one of the following conditions:
    //  - update_handler returns false
    //  - the current measurement times out
    //  - the health checks fail (brownout, driver fault line)
    //  - update_handler doesn't update the modulation timings in time
    //    This criterion is ignored if current_state is AXIS_STATE_IDLE
    //
    // If update_handler is going to update the motor timings, you must call motor.arm()
    // shortly before this function.
    //
    // If the function returns, it is guaranteed that error is non-zero, except if the cause
    // for the exit was a negative return value of update_handler or an external
    // state change request (requested_state != AXIS_STATE_DONT_CARE).
    // Under all exit conditions the motor is disarmed and the brake current set to zero.
    // Furthermore, if the update_handler does not set the phase voltages in time, they will
    // go to zero.
    //
    // @tparam T Must be a callable type that takes no arguments and returns a bool
    template<typename T>
    void run_control_loop(const T& update_handler) {
        while (requested_state_ == AXIS_STATE_UNDEFINED) {
            if (!do_checks()) // look for errors at axis level and also all subcomponents
                break;
            if (!do_updates()) // Update all estimators
                break;

            // Run main loop function, defer quitting for after wait
            // TODO: change arming logic to arm after waiting
            bool main_continue = update_handler();

            // Check we meet deadlines after queueing
            ++loop_counter_;

            // Wait until the current measurement interrupt fires
            if (!wait_for_current_meas()) {
                // maybe the interrupt handler is dead, let's be
                // safe and float the phases
                safety_critical_disarm_motor_pwm(motor_);
                update_brake_current();
                error_ |= ERROR_CURRENT_MEASUREMENT_TIMEOUT;
                break;
            }

            if (!main_continue)
                break;
        }
    }

    bool run_sensorless_spin_up();
    bool run_sensorless_control_loop();
    bool run_closed_loop_control_loop();
    bool run_idle_loop();

    void run_state_machine_loop();

    const AxisHardwareConfig_t& hw_config_;
    AxisConfig_t& config_;

    Encoder& encoder_;
    SensorlessEstimator& sensorless_estimator_;
    Controller& controller_;
    Motor& motor_;

    osThreadId thread_id_;
    volatile bool thread_id_valid_ = false;

    // variables exposed on protocol
    Error_t error_ = ERROR_NONE;
    bool enable_step_dir_ = false; // auto enabled after calibration, based on config.enable_step_dir
    AxisState_t requested_state_ = AXIS_STATE_STARTUP_SEQUENCE;
    AxisState_t task_chain_[10] = { AXIS_STATE_UNDEFINED };
    AxisState_t& current_state_ = task_chain_[0];
    uint32_t loop_counter_ = 0;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_property("enable_step_dir", &enable_step_dir_),
            make_protocol_ro_property("current_state", &current_state_),
            make_protocol_property("requested_state", &requested_state_),
            make_protocol_ro_property("loop_counter", &loop_counter_),
            make_protocol_object("config",
                make_protocol_property("startup_motor_calibration", &config_.startup_motor_calibration),
                make_protocol_property("startup_encoder_index_search", &config_.startup_encoder_index_search),
                make_protocol_property("startup_encoder_offset_calibration", &config_.startup_encoder_offset_calibration),
                make_protocol_property("startup_closed_loop_control", &config_.startup_closed_loop_control),
                make_protocol_property("startup_sensorless_control", &config_.startup_sensorless_control),
                make_protocol_property("enable_step_dir", &config_.enable_step_dir),
                make_protocol_property("counts_per_step", &config_.counts_per_step),
                make_protocol_property("ramp_up_time", &config_.ramp_up_time),
                make_protocol_property("ramp_up_distance", &config_.ramp_up_distance),
                make_protocol_property("spin_up_current", &config_.spin_up_current),
                make_protocol_property("spin_up_acceleration", &config_.spin_up_acceleration),
                make_protocol_property("spin_up_target_vel", &config_.spin_up_target_vel)
            ),
            make_protocol_object("motor", motor_.make_protocol_definitions()),
            make_protocol_object("controller", controller_.make_protocol_definitions()),
            make_protocol_object("encoder", encoder_.make_protocol_definitions()),
            make_protocol_object("sensorless_estimator", sensorless_estimator_.make_protocol_definitions())
        );
    }
};


DEFINE_ENUM_FLAG_OPERATORS(Axis::Error_t)

#endif /* __AXIS_HPP */
