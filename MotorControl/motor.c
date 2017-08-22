#include <stm32f4xx_hal.h> // Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4
#include <arm_math.h>

#include <motor.h>

#include <main.h>
#include <tim.h>
#include <spi.h>
#include <utils.h>

const int num_motors = sizeof(motors) / sizeof(motors[0]);

// TODO: Migrate to C++, clearly we are actually doing object oriented code here...
// TODO: For nice encapsulation, consider not having the motor objects public
Motor_t motors[] = {
    {
        // M0
        .control_mode = AXIS_0_CONTROL_MODE, //see: Motor_control_mode_t
        .enable_step_dir = false,                   //auto enabled after calibration
        .counts_per_step = 2.0f,
        .error = ERROR_NO_ERROR,
        .pos_setpoint = 0.0f,
        .pos_gain = 20.0f, // [(counts/s) / counts]
        // .vel_setpoint = 40000.0f,
        .vel_setpoint = 800.0f,
        // .vel_gain = 15.0f / 10000.0f, // [A/(counts/s)]
        .vel_gain = 15.0f / 200.0f, // [A/(rad/s)]
        // .vel_integrator_gain = 10.0f / 10000.0f, // [A/(counts/s * s)]
        .vel_integrator_gain = 0.0f,    // [A/(rad/s * s)]
        .vel_integrator_current = 0.0f, // [A]
        .vel_limit = 80000.0f,          // [counts/s]
        .current_setpoint = 0.0f,       // [A]
        .calibration_current = 10.0f,   // [A]
        .phase_inductance = 0.0f,       // to be set by measure_phase_inductance
        .phase_resistance = 0.0f,       // to be set by measure_phase_resistance
        .motor_thread = 0,
        .thread_ready = false,
        .enable_control = true,
        .do_calibration = true,
        .calibration_ok = false,
        .motor_timer = &htim1,
        .next_timings = {TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2},
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
        .shunt_conductance = 1.0f / 0.0005f, //[S]
        .phase_current_rev_gain = 0.0f,      // to be set by DRV8301_setup
        .current_control = {
            // .current_lim = 75.0f, //[A] // Note: consistent with 40v/v gain
            .current_lim = 10.0f, //[A]
            .p_gain = 0.0f,       // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f,       // [V/As] should be auto set after resistance and inductance measurement
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
            .encoder_timer = &htim3, .encoder_offset = 0, .encoder_state = 0,
            .motor_dir = 0,  // set by calib_enc_offset
            .phase = 0.0f,   // [rad]
            .pll_pos = 0.0f, // [rad]
            .pll_vel = 0.0f, // [rad/s]
            .pll_kp = 0.0f,  // [rad/s / rad]
            .pll_ki = 0.0f,  // [(rad/s^2) / rad]
        },
        .sensorless = {
            .phase = 0.0f,                       // [rad]
            .pll_pos = 0.0f,                     // [rad]
            .pll_vel = 0.0f,                     // [rad/s]
            .pll_kp = 0.0f,                      // [rad/s / rad]
            .pll_ki = 0.0f,                      // [(rad/s^2) / rad]
            .observer_gain = 1000.0f,            // [rad/s]
            .flux_state = {0.0f, 0.0f},          // [Vs]
            .V_alpha_beta_memory = {0.0f, 0.0f}, // [V]
            .pm_flux_linkage = 1.58e-3f,         // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
            .estimator_good = false,
            .spin_up_current = 10.0f,       // [A]
            .spin_up_acceleration = 400.0f, // [rad/s^2]
            .spin_up_target_vel = 400.0f,   // [rad/s]
        },
        .timing_log_index = 0,
        .timing_log = {0},
    },
    {                                            // M1
     .control_mode = CTRL_MODE_VELOCITY_CONTROL, //see: Motor_control_mode_t
     .enable_step_dir = false,                   //auto enabled after calibration
     .counts_per_step = 2.0f,
     .error = ERROR_NO_ERROR,
     .pos_setpoint = 0.0f,
     .pos_gain = 20.0f, // [(counts/s) / counts]
     .vel_setpoint = 0.0f,
     .vel_gain = 15.0f / 10000.0f,            // [A/(counts/s)]
     .vel_integrator_gain = 10.0f / 10000.0f, // [A/(counts/s * s)]
     .vel_integrator_current = 0.0f,          // [A]
     .vel_limit = 20000.0f,                   // [counts/s]
     .current_setpoint = 0.0f,                // [A]
     .calibration_current = 10.0f,            // [A]
     .phase_inductance = 0.0f,                // to be set by measure_phase_inductance
     .phase_resistance = 0.0f,                // to be set by measure_phase_resistance
     .motor_thread = 0,
     .thread_ready = false,
     .enable_control = false,
     .do_calibration = false,
     .calibration_ok = false,
     .motor_timer = &htim8,
     .next_timings = {TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2},
     .control_deadline = (3 * TIM_1_8_PERIOD_CLOCKS) / 2,
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
     .shunt_conductance = 1.0f / 0.0005f, //[S]
     .phase_current_rev_gain = 0.0f,      // to be set by DRV8301_setup
     .current_control = {
         // .current_lim = 75.0f, //[A] // Note: consistent with 40v/v gain
         .current_lim = 10.0f, //[A]
         .p_gain = 0.0f,       // [V/A] should be auto set after resistance and inductance measurement
         .i_gain = 0.0f,       // [V/As] should be auto set after resistance and inductance measurement
         .v_current_control_integral_d = 0.0f,
         .v_current_control_integral_q = 0.0f,
         .Ibus = 0.0f,
         .final_v_alpha = 0.0f,
         .final_v_beta = 0.0f,
     },
     .rotor_mode = ROTOR_MODE_ENCODER,
     .encoder = {
         .encoder_timer = &htim4, .encoder_offset = 0, .encoder_state = 0,
         .motor_dir = 0,  // set by calib_enc_offset
         .phase = 0.0f,   // [rad]
         .pll_pos = 0.0f, // [rad]
         .pll_vel = 0.0f, // [rad/s]
         .pll_kp = 0.0f,  // [rad/s / rad]
         .pll_ki = 0.0f,  // [(rad/s^2) / rad]
     },
     .sensorless = {
         .phase = 0.0f,                       // [rad]
         .pll_pos = 0.0f,                     // [rad]
         .pll_vel = 0.0f,                     // [rad/s]
         .pll_kp = 0.0f,                      // [rad/s / rad]
         .pll_ki = 0.0f,                      // [(rad/s^2) / rad]
         .observer_gain = 1000.0f,            // [rad/s]
         .flux_state = {0.0f, 0.0f},          // [Vs]
         .V_alpha_beta_memory = {0.0f, 0.0f}, // [V]
         .pm_flux_linkage = 1.58e-3f,         // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
         .estimator_good = false,
     },
     .timing_log_index = 0,
     .timing_log = {0}}};

bool motor_calibration(Motor_t *motor)
{
    motor->calibration_ok = false;
    motor->error = ERROR_NO_ERROR;

    // #warning(hardcoded values for SK3-5065-280kv!)
    // float R = 0.0332548246f;
    // float L = 7.97315806e-06f;

    if (!measure_phase_resistance(motor, motor->calibration_current, 1.0f))
        return false;
    if (!measure_phase_inductance(motor, -1.0f, 1.0f))
        return false;
    if (motor->rotor_mode == ROTOR_MODE_ENCODER ||
        motor->rotor_mode == ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS)
    {
        if (!calib_enc_offset(motor, motor->calibration_current * motor->phase_resistance))
            return false;
    }

    calculate_current_gains(motor);
    if (!calculate_pll_gains(motor))
    {
        return false;
    }

    motor->calibration_ok = true;
    return true;
}

// Calculate current control gains
void calculate_current_gains(Motor_t *motor)
{
    float current_control_bandwidth = 1000.0f; // [rad/s]
    motor->current_control.p_gain = current_control_bandwidth * motor->phase_inductance;
    float plant_pole = motor->phase_resistance / motor->phase_inductance;
    motor->current_control.i_gain = plant_pole * motor->current_control.p_gain;
}

// Calculate encoder pll gains
bool calculate_pll_gains(Motor_t *motor)
{
    float encoder_pll_bandwidth = 1000.0f; // [rad/s]
    motor->encoder.pll_kp = 2.0f * encoder_pll_bandwidth;

    // Check that we don't get problems with discrete time approximation
    if (!(get_current_meas_period() * motor->encoder.pll_kp < 1.0f))
    {
        motor->error = ERROR_CALIBRATION_TIMING;
        return false;
    }

    // Critically damped
    motor->encoder.pll_ki = 0.25f * (motor->encoder.pll_kp * motor->encoder.pll_kp);

    // sensorless pll same as encoder (for now)
    motor->sensorless.pll_kp = motor->encoder.pll_kp;
    motor->sensorless.pll_ki = motor->encoder.pll_ki;
    return true;
}

// TODO check Ibeta balance to verify good motor connection
// Melon Refactor - Good
bool measure_phase_resistance(Motor_t *motor, float test_current, float max_voltage)
{
    static const float kI = 10.0f;                          //[(V/s)/A]
    int num_test_cycles = 3.0f / get_current_meas_period(); // Test runs for 3s
    float test_voltage = 0.0f;
    for (int i = 0; i < num_test_cycles; ++i)
    {
        osEvent evt = osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT);
        if (evt.status != osEventSignal)
        {
            motor->error = ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT;
            return false;
        }
        float Ialpha = -0.5f * (motor->current_meas.phB + motor->current_meas.phC);
        test_voltage += (kI * get_current_meas_period()) * (test_current - Ialpha);
        test_voltage = MACRO_CONSTRAIN(test_voltage, -max_voltage, max_voltage);

        // Test voltage along phase A
        queue_voltage_timings(motor, test_voltage, 0.0f);

        // Check we meet deadlines after queueing
        motor->last_cpu_time = check_timing(motor);
        if (!(motor->last_cpu_time < motor->control_deadline))
        {
            motor->error = ERROR_PHASE_RESISTANCE_TIMING;
            return false;
        }
    }

    // De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);

    float R = test_voltage / test_current;
    if (fabs(test_voltage) == fabs(max_voltage) || R < 0.01f || R > 1.0f)
    {
        motor->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        return false;
    }
    motor->phase_resistance = R;
    return true;
}

// Melon Refactor - Good
bool measure_phase_inductance(Motor_t *motor, float voltage_low, float voltage_high)
{
    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    static const int num_cycles = 5000;

    for (int t = 0; t < num_cycles; ++t)
    {
        for (int i = 0; i < 2; ++i)
        {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal)
            {
                motor->error = ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT;
                return false;
            }
            Ialphas[i] += -motor->current_meas.phB - motor->current_meas.phC;

            // Test voltage along phase A
            queue_voltage_timings(motor, test_voltages[i], 0.0f);

            // Check we meet deadlines after queueing
            motor->last_cpu_time = check_timing(motor);
            if (!(motor->last_cpu_time < motor->control_deadline))
            {
                motor->error = ERROR_PHASE_INDUCTANCE_TIMING;
                return false;
            }
        }
    }

    // De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (get_current_meas_period() * (float)num_cycles);
    float L = v_L / dI_by_dt;

    // TODO arbitrary values set for now
    if (L < 1e-6f || L > 500e-6f)
    {
        motor->error = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        return false;
    }
    motor->phase_inductance = L;
    return true;
}

// TODO: Do the scan with current, not voltage!
// TODO: add check_timing
bool calib_enc_offset(Motor_t *motor, float voltage_magnitude)
{
    static const float start_lock_duration = 1.0f;
    static const int num_steps = 1024;
    static const float dt_step = 1.0f / 500.0f;
    static const float scan_range = 4.0f * M_PI;
    const float step_size = scan_range / (float)num_steps; // TODO handle const expressions better (maybe switch to C++ ?)

    int32_t init_enc_val = (int16_t)motor->encoder.encoder_timer->Instance->CNT;
    int32_t encvaluesum = 0;

    // go to encoder zero phase for start_lock_duration to get ready to scan
    for (int i = 0; i < start_lock_duration * get_current_meas_hz(); ++i)
    {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal)
        {
            motor->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
            return false;
        }
        queue_voltage_timings(motor, voltage_magnitude, 0.0f);
    }
    // scan forwards
    for (float ph = -scan_range / 2.0f; ph < scan_range / 2.0f; ph += step_size)
    {
        for (int i = 0; i < dt_step * (float)get_current_meas_hz(); ++i)
        {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal)
            {
                motor->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);
        }
        encvaluesum += (int16_t)motor->encoder.encoder_timer->Instance->CNT;
    }
    // check direction
    if ((int16_t)motor->encoder.encoder_timer->Instance->CNT > init_enc_val + 8)
    {
        // motor same dir as encoder
        motor->encoder.motor_dir = 1;
    }
    else if ((int16_t)motor->encoder.encoder_timer->Instance->CNT < init_enc_val - 8)
    {
        // motor opposite dir as encoder
        motor->encoder.motor_dir = -1;
    }
    else
    {
        // Encoder response error
        motor->error = ERROR_ENCODER_RESPONSE;
        return false;
    }
    // scan backwards
    for (float ph = scan_range / 2.0f; ph > -scan_range / 2.0f; ph -= step_size)
    {
        for (int i = 0; i < dt_step * (float)get_current_meas_hz(); ++i)
        {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal)
            {
                motor->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);
        }
        encvaluesum += (int16_t)motor->encoder.encoder_timer->Instance->CNT;
    }

    int offset = encvaluesum / (num_steps * 2);
    motor->encoder.encoder_offset = offset;
    return true;
}

void scan_motor_loop(Motor_t *motor, float omega, float voltage_magnitude)
{
    for (;;)
    {
        for (float ph = 0.0f; ph < 2.0f * M_PI; ph += omega * get_current_meas_period())
        {
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);

            // Check we meet deadlines after queueing
            motor->last_cpu_time = check_timing(motor);
            if (!(motor->last_cpu_time < motor->control_deadline))
            {
                motor->error = ERROR_SCAN_MOTOR_TIMING;
                return;
            }
        }
    }
}

void queue_voltage_timings(Motor_t *motor, float v_alpha, float v_beta)
{
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    queue_modulation_timings(motor, mod_alpha, mod_beta);
}

void queue_modulation_timings(Motor_t *motor, float mod_alpha, float mod_beta)
{
    float tA, tB, tC;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    motor->next_timings[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    motor->next_timings[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    motor->next_timings[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
}

uint16_t check_timing(Motor_t *motor)
{
    TIM_HandleTypeDef *htim = motor->motor_timer;
    uint16_t timing = htim->Instance->CNT;
    bool down = htim->Instance->CR1 & TIM_CR1_DIR;
    if (down)
    {
        uint16_t delta = TIM_1_8_PERIOD_CLOCKS - timing;
        timing = TIM_1_8_PERIOD_CLOCKS + delta;
    }

    if (++(motor->timing_log_index) == TIMING_LOG_SIZE)
    {
        motor->timing_log_index = 0;
    }
    motor->timing_log[motor->timing_log_index] = timing;

    return timing;
}