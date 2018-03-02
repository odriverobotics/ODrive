
#include <stdlib.h>
#include <functional>
#include "gpio.h"

#include "utils.h"
#include "axis.hpp"

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

static void step_cb_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->step_cb();
}

void Axis::setup() {
    encoder.setup();
    motor.setup();
}

void Axis::start_thread() {
    osThreadDef(thread_def, run_state_machine_loop, hw_config.thread_priority, 0, 512);
    thread_id = osThreadCreate(osThread(thread_def), this);
    thread_id_valid = true;
}

void Axis::signal_thread(thread_signals sig) {
    if (thread_id_valid)
        osSignalSet(thread_id, sig);
}

// step/direction interface
void Axis::step_cb() {
    if (enable_step_dir) {
        GPIO_PinState dir_pin = HAL_GPIO_ReadPin(hw_config.dir_port, hw_config.dir_pin);
        float dir = (dir_pin == GPIO_PIN_SET) ? 1.0f : -1.0f;
        controller.pos_setpoint += dir * config.counts_per_step;
    }
};

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

//Returns true if everything is OK (no fault)
bool Axis::check_PSU_brownout() {
    if(vbus_voltage < config.dc_bus_brownout_trip_level)
        return false;
    return true;
}

// Returns true if everything is ok. Sets motor->error and returns false otherwise.
bool Axis::do_checks() {
    if (!motor.check_DRV_fault()) {
        motor.error = ERROR_DRV_FAULT;
        return false;
    }
    if (!check_PSU_brownout()) {
        motor.error = ERROR_DC_BUS_BROWNOUT;
        return false;
    }
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
            return false;
        return x < 1.0f;
    });
    if (x < 1.0f)
        return false;
    
    // Late Spin-up: accelerate
    float vel = config.ramp_up_distance / config.ramp_up_time;
    float phase = wrap_pm_pi(config.ramp_up_distance);
    run_control_loop([&](){
        vel += config.spin_up_acceleration * current_meas_period;
        phase = wrap_pm_pi(phase + vel * current_meas_period);
        float I_mag = config.spin_up_current;
        if (!motor.update(I_mag, phase))
            return false;
        return vel < config.spin_up_target_vel;
    });
    return vel >= config.spin_up_target_vel;
}

// Note run_sensorless_control_loop and run_closed_loop_control_loop are very similar and differ only in where we get the estimate from.
bool Axis::run_sensorless_control_loop() {
    run_control_loop([this](){
        float pos_estimate, vel_estimate, phase, current_setpoint;

        // We update the encoder just in case someone needs the output for testing
        encoder.update(nullptr, nullptr, nullptr);
        if (!sensorless_estimator.update(&pos_estimate, &vel_estimate, &phase))
            return false;
        if (!controller.update(pos_estimate, vel_estimate, &current_setpoint))
            return false;
        return motor.update(current_setpoint, phase);
    });
    return false;
}

bool Axis::run_closed_loop_control_loop() {
    run_control_loop([this](){
        float pos_estimate, vel_estimate, phase, current_setpoint;

        // We update the sensorless estimator just in case someone needs the output for testing
        sensorless_estimator.update(nullptr, nullptr, nullptr);
        if (!encoder.update(&pos_estimate, &vel_estimate, &phase))
            return false;
        if (!controller.update(pos_estimate, vel_estimate, &current_setpoint))
            return false;
        return motor.update(current_setpoint, phase);
    });
    return false;
}

bool Axis::run_idle_loop() {
    // TODO: allow preemption
    for (;;) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
            motor.error = ERROR_FOC_MEASUREMENT_TIMEOUT;
            break;
        }
    }
    return false;
}

void Axis::run_state_machine_loop() {

    //TODO: Move this somewhere else
    // TODO: respect changes of CPR
    // Allocate the map for anti-cogging algorithm and initialize all values to 0.0f
    int encoder_cpr = encoder.config.cpr;
    controller.anticogging.cogging_map = (float*)malloc(encoder_cpr * sizeof(float));
    if (controller.anticogging.cogging_map != NULL) {
        for (int i = 0; i < encoder_cpr; i++) {
            controller.anticogging.cogging_map[i] = 0.0f;
        }
    }

    enum AxisState_t {
        AXIS_STATE_MOTOR_CALIBRATION,
        AXIS_STATE_ENCODER_CALIBRATION,
        AXIS_STATE_SENSORLESS_SPINUP,
        AXIS_STATE_SENSORLESS_CONTROL,
        AXIS_STATE_CLOSED_LOOP_CONTROL,
        AXIS_STATE_IDLE
    };
    AxisState_t axis_state = AXIS_STATE_MOTOR_CALIBRATION;
    
    for (;;) {
        switch (axis_state) {
        case AXIS_STATE_MOTOR_CALIBRATION:
            if (!config.enable_motor_calibration || motor.run_calibration()) {
                axis_state = AXIS_STATE_ENCODER_CALIBRATION;
            } else {
                axis_state = AXIS_STATE_IDLE;
            }
            break;
        case AXIS_STATE_ENCODER_CALIBRATION:
            if (!config.enable_encoder_calibration || encoder.run_calibration()) {
                axis_state = config.enable_control ?
                            config.sensorless ?
                            AXIS_STATE_SENSORLESS_SPINUP :
                            AXIS_STATE_CLOSED_LOOP_CONTROL :
                            AXIS_STATE_IDLE;
                if (axis_state != AXIS_STATE_IDLE)
                    set_step_dir_enabled(config.enable_step_dir_after_calibration);
            } else {
                axis_state = AXIS_STATE_IDLE;
            }
            break;
        case AXIS_STATE_SENSORLESS_SPINUP:
            if (run_sensorless_spin_up()) {
                axis_state = AXIS_STATE_SENSORLESS_CONTROL;
            } else {
                axis_state = AXIS_STATE_IDLE;
            }
            break;
        case AXIS_STATE_SENSORLESS_CONTROL:
            run_sensorless_control_loop();
            axis_state = AXIS_STATE_IDLE; // TODO: restart if desired
            break;
        case AXIS_STATE_CLOSED_LOOP_CONTROL:
            run_closed_loop_control_loop();
            axis_state = AXIS_STATE_IDLE;
            break;
        case AXIS_STATE_IDLE:
        default:
            run_idle_loop();
            break;
        }
    }

/*
    bool calibration_ok = false;
    for (;;) {
        // Keep rotor estimation up to date while idling
        osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
        loop_updates(legacy_motor_ref_);

        if (do_calibration_) {
            do_calibration_ = false;
            calibration_ok = motor.do_calibration();
            if (calibration_ok)
                calibration_ok = encoder.do_calibration();
        }

        if (calibration_ok && enable_control_) {
            enable_step_dir = true;
            if (rotor_mode == ROTOR_MODE_SENSORLESS) {
                bool spin_up_ok = do_sensorless_spin_up();
                if (spin_up_ok)
                    do_sensorless_control();
            } else {
                do_closed_loop_control();
            }

            if (enable_control_) {  // if control is still enabled, we exited because of error
                calibration_ok = false;
                enable_control_ = false;
            }
        }
    }*/
    thread_id_valid = false;
}
