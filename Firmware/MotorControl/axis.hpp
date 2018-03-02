#ifndef __AXIS_HPP
#define __AXIS_HPP

#include <utils.h>
#include <cmsis_os.h>
#include <functional>

#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4 // TODO: might change in future board versions
#include <arm_math.h>


#include <board_config_v3.3.h>

/*class Estimator {
public:
    virtual float get_position_estimation(void);
    virtual float get_velocity_estimation(void);
};*/
/*
class Motor {
public:
    // @brief Updates the current control loop
    virtual void update(float current_ref);
};*/

// The Axis declaration is needed in the other header files
class Axis;

#include <encoder.hpp>
#include <sensorless_estimator.hpp>
#include <controller.hpp>
#include <motor.hpp>
#include <low_level.h>

//default timeout waiting for phase measurement signals
#define PH_CURRENT_MEAS_TIMEOUT 2 // [ms]

static const float current_meas_period = CURRENT_MEAS_PERIOD;
static const int current_meas_hz = CURRENT_MEAS_HZ;
extern float vbus_voltage;
extern float brake_resistance;     // [ohm]

constexpr size_t AXIS_COUNT = 2;
extern Axis *axes[AXIS_COUNT];

/*
class Controller {
public:
    // @brief Updates the controller loop(s)
    virtual void update(void);
};*/


//Outside axis:
    //command handler
    //callback dispatch

typedef enum {
    ROTOR_MODE_ENCODER,
    ROTOR_MODE_SENSORLESS,
    ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS //Run on encoder, but still run estimator for testing
} Rotor_mode_t;

// TODO: decide if we want to consolidate all default configs in one file for ease of use?
struct AxisConfig_t {
    bool enable_motor_calibration = true;
    bool enable_encoder_calibration = true;
    bool enable_control = true;
    bool sensorless = false;
    bool enable_step_dir_after_calibration = true; // For M0 this has no effect if enable_uart is true
    float counts_per_step = 2.0f;
    float dc_bus_brownout_trip_level = 8.0f; // [V]
    Rotor_mode_t rotor_mode = ROTOR_MODE_ENCODER;

    // Spinup settings
    float ramp_up_time = 0.4f;            // [s]
    float ramp_up_distance = 4 * M_PI;    // [rad]
    float spin_up_current = 10.0f;        // [A]
    float spin_up_acceleration = 400.0f;  // [rad/s^2]
    float spin_up_target_vel = 400.0f;    // [rad/s]
};

class Axis {
public:
    //thread/os/system management
        //timing log
        //thread id
        //etc.
    //state machine
        //control mode
        //control_en/calib_ok
        //error state
    //motor
        //current controller
            //contains rotor phase logic
        //motor level calibration routines
        //low_level (implementation specifics)
            //DRV driver
            //adc callback handling
            //pwm queueing
    //rotor estimator
        //kick out rotor phase logic
    //pos/vel controller
    //step/dir handler

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

    // Infinite loop that does calibration and enters main control loop as appropriate
    void run_state_machine_loop();
    static void run_state_machine_loop(const void* ctx) {
        const_cast<Axis*>(reinterpret_cast<const Axis*>(ctx))->run_state_machine_loop();
    };

    void step_cb();
    void set_step_dir_enabled(bool enable);

    bool check_DRV_fault();
    bool check_PSU_brownout();
    bool do_checks();

    // TODO: check if this uses dynamic memory


    // @brief Runs the update handler at the frequency of the current measurements.
    //
    // The loop runs until one of the following conditions:
    //  - the update handler returns false
    //  - the current measurement times out
    //  - do_checks() becomes false
    //  - update_handler doesn't finish in time
    //
    // The function arms the motor at the beginning of the control loop and disarms it at
    // the end of the control loop.
    // Note that if this function returns, this should generally be considered an error condition,
    // unless the termination was deliberately caused by the update_handler because it was of the
    // opinion that the loop's task was completed.
    // @tparam T Must be a callable type that takes no arguments and returns a bool
    template<typename T>
    void run_control_loop(const T& update_handler) {
        motor.arm();
        while (true /*enable_control*/) { // TODO: check for state change
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
                motor.error = ERROR_FOC_MEASUREMENT_TIMEOUT;
                break;
            }
            
            // Proactively set phase voltages to 0. If the control deadline is missed,
            // the voltages will go to zero.
            motor.enqueue_voltage_timings(0.0f, 0.0f);

            if (!do_checks())
                break;

            if (!update_handler())
                break;

            update_brake_current();

            // Check we meet deadlines after queueing
            motor.last_cpu_time = motor.check_timing();
            if (!(motor.last_cpu_time < motor.hw_config.control_deadline)) {
                motor.error = ERROR_PHASE_RESISTANCE_TIMING;
                break;
            }
            ++loop_counter;

            // TODO: maybe we should just abort automatically as soon as error is set
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

    const AxisHardwareConfig_t& hw_config;
    AxisConfig_t& config;

    Encoder& encoder;
    SensorlessEstimator& sensorless_estimator;
    Controller& controller;
    Motor& motor;

    osThreadId thread_id;
    volatile bool thread_id_valid = false;
    bool enable_step_dir = false;                    //auto enabled after calibration
    uint32_t loop_counter = 0;
};

#endif /* __AXIS_HPP */
