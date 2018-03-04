#ifndef __AXIS_HPP
#define __AXIS_HPP

#ifndef __ODRIVE_MAIN_HPP
#error "This file should not be included directly. Include odrive_main.hpp instead."
#endif


enum AxisState_t {
    AXIS_STATE_STARTUP,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_CALIBRATION,
    AXIS_STATE_SENSORLESS_SPINUP,
    AXIS_STATE_SENSORLESS_CONTROL,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATE_DONT_CARE // used to indicate that no state request is pending
};

struct AxisConfig_t {
    bool enable_motor_calibration = true;   //<! run motor calibration at startup, skip otherwise
    bool enable_encoder_calibration = true; //<! run encoder calibration after startup, skip otherwise
    bool enable_closed_loop_control = true; //<! enable closed loop control after calibration/startup
    bool enable_sensorless_control = false; //<! enable sensorless control after calibration/startup
    bool enable_step_dir = true; //<! enable step/dir input after calibration
                                 //   For M0 this has no effect if enable_uart is true

    float counts_per_step = 2.0f;
    float dc_bus_brownout_trip_level = 8.0f; //<! [V] voltage at which the motor stops operating

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
        ERROR_NO_ERROR,
        ERROR_BAD_VOLTAGE,
        ERROR_CURRENT_MEASUREMENT_TIMEOUT,
        ERROR_CONTROL_LOOP_TIMEOUT,
        ERROR_MOTOR_FAILED,
        ERROR_SENSORLESS_ESTIMATOR_FAILED,
        ERROR_ENCODER_FAILED,
        ERROR_CONTROLLER_FAILED,
        ERROR_POS_CTRL_DURING_SENSORLESS,
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
    void signal_thread(thread_signals sig);

    void step_cb();
    void set_step_dir_enabled(bool enable);

    bool check_DRV_fault();
    bool check_PSU_brownout();
    bool do_checks();

    // @brief Runs the specified update handler at the frequency of the current measurements.
    //
    // The loop runs until one of the following conditions:
    //  - the update handler returns false
    //  - the current measurement times out
    //  - the health checks fail (brownout, driver fault line)
    //  - update_handler doesn't finish in time
    //
    // The function arms the motor at the beginning of the control loop and disarms it at
    // the end of the control loop.
    //
    // If the function returns, it is guaranteed that error is non-zero, except if the cause for the exit
    // reason for the loop termination was a negative return value of update_handler or an external
    // state change request (requested_state != AXIS_STATE_DONT_CARE).
    // Under all exit conditions the motor is disarmed and the brake current set to zero.
    // Furthermore, if the update_handler does not set the phase voltages in time, they will
    // go to zero.
    //
    // @tparam T Must be a callable type that takes no arguments and returns a bool
    template<typename T>
    void run_control_loop(const T& update_handler) {
        motor.arm();
        while (requested_state == AXIS_STATE_DONT_CARE
            && error == ERROR_NO_ERROR /* error may be set by interrupt handler */ ) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
                error = ERROR_CURRENT_MEASUREMENT_TIMEOUT;
                break;
            }
            
            // Proactively set phase voltages to 0. If the control deadline is missed,
            // the voltages will go to zero.
            motor.enqueue_voltage_timings(0.0f, 0.0f);

            if (!do_checks()) // error set during function call
                break;

            if (!update_handler()) // error set during function call
                break;

            update_brake_current();

            // Check we meet deadlines after queueing
            motor.last_cpu_time = motor.check_timing();
            if (!(motor.last_cpu_time < motor.hw_config.control_deadline)) {
                error = ERROR_CONTROL_LOOP_TIMEOUT;
                break;
            }
            ++loop_counter;
        }

        // We are exiting control: disarm motor, reset Ibus, and update brake current
        motor.disarm();
        motor.current_control.Ibus = 0.0f;
        update_brake_current();
    }

    bool run_sensorless_spin_up();
    bool run_sensorless_control_loop();
    bool run_closed_loop_control_loop();
    bool run_idle_loop();

    void run_state_machine_loop();

    const AxisHardwareConfig_t& hw_config;
    AxisConfig_t& config;

    Encoder& encoder;
    SensorlessEstimator& sensorless_estimator;
    Controller& controller;
    Motor& motor;

    Error_t error = ERROR_NO_ERROR;
    osThreadId thread_id;
    volatile bool thread_id_valid = false;
    bool enable_step_dir = false; // auto enabled after calibration, based on enable_step_dir_after_calibration
    AxisState_t current_state = AXIS_STATE_STARTUP;
    AxisState_t requested_state = AXIS_STATE_DONT_CARE;
    uint32_t loop_counter = 0;
};

#endif /* __AXIS_HPP */
