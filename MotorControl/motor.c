#include <motor.h>

#include <tim.h>
#include <spi.h>

// TODO: Migrate to C++, clearly we are actually doing object oriented code here...
// TODO: For nice encapsulation, consider not having the motor objects public
Motor_t motors[] = {
    {   // M0
        .control_mode = CTRL_MODE_VELOCITY_CONTROL, //see: Motor_control_mode_t
        .enable_step_dir = false, //auto enabled after calibration
        .counts_per_step = 2.0f,
        .error = ERROR_NO_ERROR,
        .pos_setpoint = 0.0f,
        .pos_gain = 20.0f, // [(counts/s) / counts]
        // .vel_setpoint = 40000.0f,
        .vel_setpoint = 800.0f,
        // .vel_gain = 15.0f / 10000.0f, // [A/(counts/s)]
        .vel_gain = 15.0f / 200.0f, // [A/(rad/s)]
        // .vel_integrator_gain = 10.0f / 10000.0f, // [A/(counts/s * s)]
        .vel_integrator_gain = 0.0f, // [A/(rad/s * s)]
        .vel_integrator_current = 0.0f, // [A]
        .vel_limit = 80000.0f, // [counts/s]
        .current_setpoint = 0.0f, // [A]
        .calibration_current = 10.0f, // [A]
        .phase_inductance = 0.0f, // to be set by measure_phase_inductance
        .phase_resistance = 0.0f, // to be set by measure_phase_resistance
        .motor_thread = 0,
        .thread_ready = false,
        .enable_control = true,
        .do_calibration = true,
        .calibration_ok = false,
        .motor_timer = &htim1,
        .next_timings = {TIM_1_8_PERIOD_CLOCKS/2, TIM_1_8_PERIOD_CLOCKS/2, TIM_1_8_PERIOD_CLOCKS/2},
        .control_deadline = TIM_1_8_PERIOD_CLOCKS,
        .last_cpu_time = 0,
        .current_meas = {0.0f, 0.0f},
        .DC_calib = {0.0f, 0.0f},
        .gate_driver = {
            .spiHandle = &hspi3,
            // Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M0_nCS_GPIO_Port,
            .nCSgpioNumber = M0_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false,
        },
        // .gate_driver_regs Init by DRV8301_setup
        .shunt_conductance = 1.0f/0.0005f, //[S]
        .phase_current_rev_gain = 0.0f, // to be set by DRV8301_setup
        .current_control = {
            // .current_lim = 75.0f, //[A] // Note: consistent with 40v/v gain
            .current_lim = 10.0f, //[A]
            .p_gain = 0.0f, // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f, // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f,
            .Ibus = 0.0f,
            .final_v_alpha = 0.0f,
            .final_v_beta = 0.0f,
        },
        // .rotor_mode = ROTOR_MODE_ENCODER,
        // .rotor_mode = ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS,
        .rotor_mode = ROTOR_MODE_SENSORLESS,
        .encoder = {
            .encoder_timer = &htim3,
            .encoder_offset = 0,
            .encoder_state = 0,
            .motor_dir = 0, // set by calib_enc_offset
            .phase = 0.0f, // [rad]
            .pll_pos = 0.0f, // [rad]
            .pll_vel = 0.0f, // [rad/s]
            .pll_kp = 0.0f, // [rad/s / rad]
            .pll_ki = 0.0f, // [(rad/s^2) / rad]
        },
        .sensorless = {
            .phase = 0.0f, // [rad]
            .pll_pos = 0.0f, // [rad]
            .pll_vel = 0.0f, // [rad/s]
            .pll_kp = 0.0f, // [rad/s / rad]
            .pll_ki = 0.0f, // [(rad/s^2) / rad]
            .observer_gain = 1000.0f, // [rad/s]
            .flux_state = {0.0f, 0.0f}, // [Vs]
            .V_alpha_beta_memory = {0.0f, 0.0f}, // [V]
            .pm_flux_linkage = 1.58e-3f, // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
            .estimator_good = false,
            .spin_up_current = 10.0f, // [A]
            .spin_up_acceleration = 400.0f, // [rad/s^2]
            .spin_up_target_vel = 400.0f, // [rad/s]
        },
        .timing_log_index = 0,
        .timing_log = {0},
    },
    {   // M1
        .control_mode = CTRL_MODE_VELOCITY_CONTROL, //see: Motor_control_mode_t
        .enable_step_dir = false, //auto enabled after calibration
        .counts_per_step = 2.0f,
        .error = ERROR_NO_ERROR,
        .pos_setpoint = 0.0f,
        .pos_gain = 20.0f, // [(counts/s) / counts]
        .vel_setpoint = 0.0f,
        .vel_gain = 15.0f / 10000.0f, // [A/(counts/s)]
        .vel_integrator_gain = 10.0f / 10000.0f, // [A/(counts/s * s)]
        .vel_integrator_current = 0.0f, // [A]
        .vel_limit = 20000.0f, // [counts/s]
        .current_setpoint = 0.0f, // [A]
        .calibration_current = 10.0f, // [A]
        .phase_inductance = 0.0f, // to be set by measure_phase_inductance
        .phase_resistance = 0.0f, // to be set by measure_phase_resistance
        .motor_thread = 0,
        .thread_ready = false,
        .enable_control = false,
        .do_calibration = false,
        .calibration_ok = false,
        .motor_timer = &htim8,
        .next_timings = {TIM_1_8_PERIOD_CLOCKS/2, TIM_1_8_PERIOD_CLOCKS/2, TIM_1_8_PERIOD_CLOCKS/2},
        .control_deadline = (3*TIM_1_8_PERIOD_CLOCKS)/2,
        .last_cpu_time = 0,
        .current_meas = {0.0f, 0.0f},
        .DC_calib = {0.0f, 0.0f},
        .gate_driver = {
            .spiHandle = &hspi3,
            // Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M1_nCS_GPIO_Port,
            .nCSgpioNumber = M1_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false,
        },
        // .gate_driver_regs Init by DRV8301_setup
        .shunt_conductance = 1.0f/0.0005f, //[S]
        .phase_current_rev_gain = 0.0f, // to be set by DRV8301_setup
        .current_control = {
            // .current_lim = 75.0f, //[A] // Note: consistent with 40v/v gain
            .current_lim = 10.0f, //[A]
            .p_gain = 0.0f, // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f, // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f,
            .Ibus = 0.0f,
            .final_v_alpha = 0.0f,
            .final_v_beta = 0.0f,
        },
        .rotor_mode = ROTOR_MODE_ENCODER,
        .encoder = {
            .encoder_timer = &htim4,
            .encoder_offset = 0,
            .encoder_state = 0,
            .motor_dir = 0, // set by calib_enc_offset
            .phase = 0.0f, // [rad]
            .pll_pos = 0.0f, // [rad]
            .pll_vel = 0.0f, // [rad/s]
            .pll_kp = 0.0f, // [rad/s / rad]
            .pll_ki = 0.0f, // [(rad/s^2) / rad]
        },
        .sensorless = {
            .phase = 0.0f, // [rad]
            .pll_pos = 0.0f, // [rad]
            .pll_vel = 0.0f, // [rad/s]
            .pll_kp = 0.0f, // [rad/s / rad]
            .pll_ki = 0.0f, // [(rad/s^2) / rad]
            .observer_gain = 1000.0f, // [rad/s]
            .flux_state = {0.0f, 0.0f}, // [Vs]
            .V_alpha_beta_memory = {0.0f, 0.0f}, // [V]
            .pm_flux_linkage = 1.58e-3f, // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
            .estimator_good = false,
        },
        .timing_log_index = 0,
        .timing_log = {0}
    }
};
const int num_motors = sizeof(motors)/sizeof(motors[0]);