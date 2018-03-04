
#include <stdlib.h>
#include <functional>
#include "gpio.h"

#include "utils.h"
#include "odrive_main.hpp"

Axis::Axis(const AxisHardwareConfig_t& hw_config,
           AxisConfig_t& config,
           Encoder& encoder,
           SensorlessEstimator& sensorless_estimator,
           Controller& controller,
           Motor& motor)
    : hw_config(hw_config),
      config(config),
      encoder(encoder),
      sensorless_estimator(sensorless_estimator),
      controller(controller),
      motor(motor)
{
    encoder.axis = this;
    sensorless_estimator.axis = this;
    controller.axis = this;
    motor.axis = this;
}

// @brief Sets up all components of the axis,
// such as gate driver and encoder hardware.
void Axis::setup() {
    encoder.setup();
    motor.setup();
}

static void run_state_machine_loop_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->run_state_machine_loop();
}

// @brief Starts run_state_machine_loop in a new thread
void Axis::start_thread() {
    osThreadDef(thread_def, run_state_machine_loop_wrapper, hw_config.thread_priority, 0, 512);
    thread_id = osThreadCreate(osThread(thread_def), this);
    thread_id_valid = true;
}

// @brief Unblocks the control loop thread.
// This is called from the current sense interrupt handler.
void Axis::signal_thread(thread_signals sig) {
    if (thread_id_valid)
        osSignalSet(thread_id, sig);
}

static void step_cb_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->step_cb();
}

// step/direction interface
void Axis::step_cb() {
    if (enable_step_dir) {
        GPIO_PinState dir_pin = HAL_GPIO_ReadPin(hw_config.dir_port, hw_config.dir_pin);
        float dir = (dir_pin == GPIO_PIN_SET) ? 1.0f : -1.0f;
        controller.pos_setpoint += dir * config.counts_per_step;
    }
};

// @brief Enables or disables step/dir input
void Axis::set_step_dir_enabled(bool enable) {
    if (enable) {
        // Set up the direction GPIO as input
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = hw_config.dir_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(hw_config.dir_port, &GPIO_InitStruct);

        // Subscribe to rising edges of the step GPIO
        GPIO_subscribe(hw_config.step_port, hw_config.step_pin, GPIO_PULLDOWN,
                step_cb_wrapper, this);

        enable_step_dir = true;
    } else {
        enable_step_dir = false;

        // Unsubscribe from step GPIO
        GPIO_unsubscribe(hw_config.step_port, hw_config.step_pin);
    }
}

// @brief Returns true if the power supply is within range
bool Axis::check_PSU_brownout() {
    if(vbus_voltage < config.dc_bus_brownout_trip_level)
        return error = ERROR_BAD_VOLTAGE, false;
    return true;
}

// @brief Returns true if everything is ok.
// Sets error and returns false otherwise.
bool Axis::do_checks() {
    if (!motor.do_checks())
        return error = ERROR_MOTOR_FAILED, false;
    if (!check_PSU_brownout())
        return error = ERROR_BAD_VOLTAGE, false;
    return true;
}

bool Axis::run_sensorless_spin_up() {
    // Early Spin-up: spiral up current
    float x = 0.0f;
    run_control_loop([&](){
        float phase = wrap_pm_pi(config.ramp_up_distance * x);
        float I_mag = config.spin_up_current * x;
        x += current_meas_period / config.ramp_up_time;
        if (!motor.update(I_mag, phase))
            return error = ERROR_MOTOR_FAILED, false;
        return x < 1.0f;
    });
    if (error != ERROR_NO_ERROR)
        return false;
    
    // Late Spin-up: accelerate
    float vel = config.ramp_up_distance / config.ramp_up_time;
    float phase = wrap_pm_pi(config.ramp_up_distance);
    run_control_loop([&](){
        vel += config.spin_up_acceleration * current_meas_period;
        phase = wrap_pm_pi(phase + vel * current_meas_period);
        float I_mag = config.spin_up_current;
        if (!motor.update(I_mag, phase))
            return error = ERROR_MOTOR_FAILED, false;
        return vel < config.spin_up_target_vel;
    });
    return error == ERROR_NO_ERROR;
}

// Note run_sensorless_control_loop and run_closed_loop_control_loop are very similar and differ only in where we get the estimate from.
bool Axis::run_sensorless_control_loop() {
    run_control_loop([this](){
        float pos_estimate, vel_estimate, phase, current_setpoint;

        if (controller.config.control_mode >= CTRL_MODE_POSITION_CONTROL)
            return error = ERROR_POS_CTRL_DURING_SENSORLESS, false;

        // We update the encoder just in case someone needs the output for testing
        encoder.update(nullptr, nullptr, nullptr);
        if (!sensorless_estimator.update(&pos_estimate, &vel_estimate, &phase))
            return error = ERROR_SENSORLESS_ESTIMATOR_FAILED, false;
        if (!controller.update(pos_estimate, vel_estimate, &current_setpoint))
            return error = ERROR_CONTROLLER_FAILED, false;
        if (!motor.update(current_setpoint, phase))
            return error = ERROR_MOTOR_FAILED, false;
        return true;
    });
    return error == ERROR_NO_ERROR;
}

bool Axis::run_closed_loop_control_loop() {
    run_control_loop([this](){
        float pos_estimate, vel_estimate, phase, current_setpoint;

        // We update the sensorless estimator just in case someone needs the output for testing
        sensorless_estimator.update(nullptr, nullptr, nullptr);
        if (!encoder.update(&pos_estimate, &vel_estimate, &phase))
            return error = ERROR_ENCODER_FAILED, false;
        if (!controller.update(pos_estimate, vel_estimate, &current_setpoint))
            return error = ERROR_CONTROLLER_FAILED, false;
        if (!motor.update(current_setpoint, phase))
            return error = ERROR_MOTOR_FAILED, false;
        return true;
    });
    return error == ERROR_NO_ERROR;
}

bool Axis::run_idle_loop() {
    while (requested_state == AXIS_STATE_DONT_CARE) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal)
            return error = ERROR_CURRENT_MEASUREMENT_TIMEOUT, false;
    }
    return error == ERROR_NO_ERROR;
}

// Infinite loop that does calibration and enters main control loop as appropriate
void Axis::run_state_machine_loop() {

    // Allocate the map for anti-cogging algorithm and initialize all values to 0.0f
    // TODO: Move this somewhere else
    // TODO: respect changes of CPR
    int encoder_cpr = encoder.config.cpr;
    controller.anticogging.cogging_map = (float*)malloc(encoder_cpr * sizeof(float));
    if (controller.anticogging.cogging_map != NULL) {
        for (int i = 0; i < encoder_cpr; i++) {
            controller.anticogging.cogging_map[i] = 0.0f;
        }
    }

    current_state = AXIS_STATE_MOTOR_CALIBRATION;
    bool force_state = false;
    
    for (;;) {
        AxisState_t next_state = AXIS_STATE_DONT_CARE;

        switch (current_state) {

        case AXIS_STATE_MOTOR_CALIBRATION:
            {
                bool skip = !force_state && !config.enable_motor_calibration;
                if (skip || motor.run_calibration()) {
                    next_state = AXIS_STATE_ENCODER_CALIBRATION;
                } else {
                    next_state = AXIS_STATE_IDLE;
                }
            }
            break;

        case AXIS_STATE_ENCODER_CALIBRATION:
            {
                bool skip = !force_state && !config.enable_encoder_calibration;
                if (skip || encoder.run_calibration()) {
                    next_state = config.enable_closed_loop_control ?
                                AXIS_STATE_CLOSED_LOOP_CONTROL :
                                config.enable_sensorless_control ?
                                AXIS_STATE_SENSORLESS_SPINUP :
                                AXIS_STATE_IDLE;
                    if (next_state != AXIS_STATE_IDLE)
                        set_step_dir_enabled(config.enable_step_dir);
                } else {
                    next_state = AXIS_STATE_IDLE;
                }
            }
            break;

        case AXIS_STATE_SENSORLESS_SPINUP:
            if (run_sensorless_spin_up()) {
                next_state = AXIS_STATE_SENSORLESS_CONTROL;
            } else {
                next_state = AXIS_STATE_IDLE;
            }
            break;

        case AXIS_STATE_SENSORLESS_CONTROL:
            run_sensorless_control_loop();
            next_state = AXIS_STATE_IDLE; // TODO: restart if desired
            break;

        case AXIS_STATE_CLOSED_LOOP_CONTROL:
            run_closed_loop_control_loop();
            next_state = AXIS_STATE_IDLE;
            break;

        case AXIS_STATE_IDLE:
        default:
            current_state = AXIS_STATE_IDLE;
            run_idle_loop();
            break;
        }

        if (requested_state != AXIS_STATE_DONT_CARE) {
            current_state = requested_state;
            requested_state = AXIS_STATE_DONT_CARE;
            force_state = true;
        } else {
            current_state = next_state;
            force_state = false;
        }
    }

    thread_id_valid = false;
}
