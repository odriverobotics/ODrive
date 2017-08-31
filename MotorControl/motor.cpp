#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4
#include <arm_math.h>

#include <motor.h>

#include <main.h>
#include <spi.h>
#include <tim.h>
#include <utils.h>



// Need to put this in wherever we're declaring the motor objects
/* DRV8301_Obj gateDriver0 = {
    .spiHandle = &hspi3,
    // Note: this board has the EN_Gate pin shared!
    .EngpioHandle = EN_GATE_GPIO_Port,
    .EngpioNumber = EN_GATE_Pin,
    .nCSgpioHandle = M0_nCS_GPIO_Port,
    .nCSgpioNumber = M0_nCS_Pin,
    .RxTimeOut = false,
    .enableTimeOut = false,
}

DRV8301_Obj gateDriver1 = {
    .spiHandle = &hspi3,
    // Note: this board has the EN_Gate pin shared!
    .EngpioHandle = EN_GATE_GPIO_Port,
    .EngpioNumber = EN_GATE_Pin,
    .nCSgpioHandle = M1_nCS_GPIO_Port,
    .nCSgpioNumber = M1_nCS_Pin,
    .RxTimeOut = false,
    .enableTimeOut = false,
} */

/* Current_Control_t current_control = {
    // .current_lim = 75.0f, //[A] // Note: consistent with 40v/v
    // gain
    .current_lim = 10.0f,  //[A]
    .p_gain = 0.0f,        // [V/A] should be auto set after resistance
                            // and inductance measurement
    .i_gain = 0.0f,        // [V/As] should be auto set after resistance
                            // and inductance measurement
    .v_current_control_integral_d = 0.0f,
    .v_current_control_integral_q = 0.0f,
    .Ibus = 0.0f,
    .final_v_alpha = 0.0f,
    .final_v_beta = 0.0f,
} */

/* Encoder encoder = {
    .encoder_timer = &htim3,
    .encoder_offset = 0,
    .encoder_state = 0,
    .motor_dir = 0,   // set by calib_enc_offset
    .phase = 0.0f,    // [rad]
    .pll_pos = 0.0f,  // [rad]
    .pll_vel = 0.0f,  // [rad/s]
    .pll_kp = 0.0f,   // [rad/s / rad]
    .pll_ki = 0.0f,   // [(rad/s^2) / rad]
} */

/* Sensorless sensorless = {
    .phase = 0.0f,                        // [rad]
    .pll_pos = 0.0f,                      // [rad]
    .pll_vel = 0.0f,                      // [rad/s]
    .pll_kp = 0.0f,                       // [rad/s / rad]
    .pll_ki = 0.0f,                       // [(rad/s^2) / rad]
    .observer_gain = 1000.0f,             // [rad/s]
    .flux_state = {0.0f, 0.0f},           // [Vs]
    .V_alpha_beta_memory = {0.0f, 0.0f},  // [V]
    .pm_flux_linkage = 1.58e-3f,          // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
    .estimator_good = false,
    .spin_up_current = 10.0f,        // [A]
    .spin_up_acceleration = 400.0f,  // [rad/s^2]
    .spin_up_target_vel = 400.0f,    // [rad/s]
} */
static int numMotors = 0;

Motor::Motor(TIM_HandleTypeDef *motor_timer, DRV8301_Obj &gate_driver, Current_control_t &current_control, Encoder &encoder, Sensorless &sensorless) {
    this->control_mode = AXIS_0_CONTROL_MODE;  // see: Motor_control_mode_t
    this->enable_step_dir = false;             // auto enabled after calibration
    this->counts_per_step = 2.0f;
    this->error = ERROR_NO_ERROR;
    this->pos_setpoint = 0.0f;
    this->pos_gain = 20.0f;  // [(counts/s) / counts]
    // .vel_setpoint = 40000.0f;
    this->vel_setpoint = 800.0f;
    // .vel_gain = 15.0f / 10000.0f; // [A/(counts/s)]
    this->vel_gain = 15.0f / 200.0f;  // [A/(rad/s)]
    // .vel_integrator_gain = 10.0f / 10000.0f; // [A/(counts/s * s)]
    this->vel_integrator_gain = 0.0f;     // [A/(rad/s * s)]
    this->vel_integrator_current = 0.0f;  // [A]
    this->vel_limit = 80000.0f;           // [counts/s]
    this->current_setpoint = 0.0f;        // [A]
    this->calibration_current = 10.0f;    // [A]
    this->phase_inductance = 0.0f;        // to be set by measure_phase_inductance
    this->phase_resistance = 0.0f;        // to be set by measure_phase_resistance
    this->motor_thread = 0;
    this->thread_ready = false;
    this->enable_control = true;
    this->do_calibration = true;
    this->calibration_ok = false;
    this->motor_timer = motor_timer;

    this->next_timings[0] = TIM_1_8_PERIOD_CLOCKS / 2;
    this->next_timings[1] = TIM_1_8_PERIOD_CLOCKS / 2;
    this->next_timings[2] = TIM_1_8_PERIOD_CLOCKS / 2;

    this->control_deadline = TIM_1_8_PERIOD_CLOCKS;
    this->last_cpu_time = 0;

    this->current_meas[0] = 0.0f;
    this->current_meas[1] = 0.0f;

    this->DC_calib = 0.0f;
    this->DC_calib = 0.0f;

    this->shunt_conductance = 1.0f / 0.0005f;  // [S]
    this->phase_current_rev_gain = 0.0f;       // to be set by DRV8301_setup

    this->rotor_mode = ROTOR_MODE_SENSORLESS;

    this->current_control = current_control;
    this->gate_driver = gate_driver;
    this->encoder = encoder;
    this->sensorless = sensorless;

    this->timing_log_index = 0;
    
    for(int i = 0; i < TIMING_LOG_SIZE; i++)
        this->timing_log[i] = 0;

    numMotors++;
}

bool calibrateMotor() {
    this->error = ERROR_NO_ERROR;

    // #warning(hardcoded values for SK3-5065-280kv!)
    // float R = 0.0332548246f;
    // float L = 7.97315806e-06f;

    if (!measure_phase_resistance(this->calibration_current, 1.0f))
        return false;
    if (!measure_phase_inductance(-1.0f, 1.0f)) return false;
    if (this->rotor_mode == ROTOR_MODE_ENCODER ||
        this->rotor_mode == ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS) {
        if (!calib_enc_offset(this->calibration_current * this->phase_resistance))
            return false;
    }

    calculateCurrentGains();
    if (!calculatePLLGains()) {
        return false;
    }

    return true;
}

// Calculate current control gains
void calculateCurrentGains() {
    float current_control_bandwidth = 1000.0f;  // [rad/s]
    this->current_control.p_gain =
        current_control_bandwidth * this->phase_inductance;
    float plant_pole = this->phase_resistance / this->phase_inductance;
    this->current_control.i_gain = plant_pole * this->current_control.p_gain;
}

// Calculate encoder pll gains
bool calculatePLLGains() {
    float encoder_pll_bandwidth = 1000.0f;  // [rad/s]
    this->encoder.pll_kp = 2.0f * encoder_pll_bandwidth;

    // Check that we don't get problems with discrete time approximation
    if (!(get_current_meas_period() * this->encoder.pll_kp < 1.0f)) {
        this->error = ERROR_CALIBRATION_TIMING;
        return false;
    }

    // Critically damped
    this->encoder.pll_ki =
        0.25f * (this->encoder.pll_kp * this->encoder.pll_kp);

    // sensorless pll same as encoder (for now)
    this->sensorless.pll_kp = this->encoder.pll_kp;
    this->sensorless.pll_ki = this->encoder.pll_ki;
    return true;
}

// TODO check Ibeta balance to verify good motor connection
bool measure_phase_resistance(float test_current, float max_voltage) {
    static const float kI = 10.0f;                           //[(V/s)/A]
    int num_test_cycles = 3.0f / get_current_meas_period();  // Test runs for 3s
    float test_voltage = 0.0f;
    for (int i = 0; i < num_test_cycles; ++i) {
        osEvent evt =
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT);
        if (evt.status != osEventSignal) {
            this->error = ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT;
            return false;
        }
        float Ialpha = -(this->current_meas.phB + this->current_meas.phC);
        test_voltage += (kI * get_current_meas_period()) * (test_current - Ialpha);
        test_voltage = MACRO_CONSTRAIN(test_voltage, -max_voltage, max_voltage);

        // Test voltage along phase A
        queue_voltage_timings(test_voltage, 0.0f);

        // Check we meet deadlines after queueing
        if (!check_deadlines()) {
            this->error = ERROR_PHASE_RESISTANCE_TIMING;
            return false;
        }
    }

    // De-energize motor
    queue_voltage_timings(0.0f, 0.0f);

    float R = test_voltage / test_current;
    if (fabs(test_voltage) == fabs(max_voltage) || R < 0.01f || R > 1.0f) {
        this->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        return false;
    }
    this->phase_resistance = R;
    return true;
}

// Melon Refactor - Good
bool Motor::measure_phase_inductance(float voltage_low, float voltage_high) {
    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    static const int num_cycles = 5000;

    for (int t = 0; t < num_cycles; ++t) {
        for (int i = 0; i < 2; ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT)
                    .status != osEventSignal) {
                this->error = ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT;
                return false;
            }
            Ialphas[i] += -this->current_meas.phB - this->current_meas.phC;

            // Test voltage along phase A
            queue_voltage_timings(test_voltages[i], 0.0f);

            // Check we meet deadlines after queueing
            if (!check_deadlines()) {
                this->error = ERROR_PHASE_INDUCTANCE_TIMING;
                return false;
            }
        }
    }

    // De-energize motor
    queue_voltage_timings(0.0f, 0.0f);

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a
    // finite timestep.
    // However, the discretisation in the current control loop inverts the same
    // discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) /
                     (get_current_meas_period() * (float)num_cycles);
    float L = v_L / dI_by_dt;

    // TODO arbitrary values set for now
    if (L < 1e-6f || L > 500e-6f) {
        this->error = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        return false;
    }
    this->phase_inductance = L;
    return true;
}

// TODO: Do the scan with current, not voltage!
// TODO: add check_timing
bool calib_enc_offset(float voltage_magnitude) {
    static const float start_lock_duration = 1.0f;
    static const int num_steps = 1024;
    static const float dt_step = 1.0f / 500.0f;
    static const float scan_range = 4.0f * M_PI;
    const float step_size =
        scan_range / (float)num_steps;  // TODO handle const expressions better
                                        // (maybe switch to C++ ?)

    int32_t init_enc_val = (int16_t)this->encoder.encoder_timer->Instance->CNT;
    int32_t encvaluesum = 0;

    // go to encoder zero phase for start_lock_duration to get ready to scan
    for (int i = 0; i < start_lock_duration * get_current_meas_hz(); ++i) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT)
                .status != osEventSignal) {
            this->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
            return false;
        }
        queue_voltage_timings(voltage_magnitude, 0.0f);
    }
    // scan forwards
    for (float ph = -scan_range / 2.0f; ph < scan_range / 2.0f; ph += step_size) {
        for (int i = 0; i < dt_step * (float)get_current_meas_hz(); ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT)
                    .status != osEventSignal) {
                this->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(v_alpha, v_beta);
        }
        encvaluesum += (int16_t)this->encoder.encoder_timer->Instance->CNT;
    }
    // check direction
    if ((int16_t)this->encoder.encoder_timer->Instance->CNT > init_enc_val + 8) {
        // motor same dir as encoder
        this->encoder.motor_dir = 1;
    } else if ((int16_t)this->encoder.encoder_timer->Instance->CNT <
               init_enc_val - 8) {
        // motor opposite dir as encoder
        this->encoder.motor_dir = -1;
    } else {
        // Encoder response error
        this->error = ERROR_ENCODER_RESPONSE;
        return false;
    }
    // scan backwards
    for (float ph = scan_range / 2.0f; ph > -scan_range / 2.0f; ph -= step_size) {
        for (int i = 0; i < dt_step * (float)get_current_meas_hz(); ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT)
                    .status != osEventSignal) {
                this->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(v_alpha, v_beta);
        }
        encvaluesum += (int16_t)this->encoder.encoder_timer->Instance->CNT;
    }

    int offset = encvaluesum / (num_steps * 2);
    this->encoder.encoder_offset = offset;
    return true;
}

void scan__loop(float omega, float voltage_magnitude) {
    for (;;) {
        for (float ph = 0.0f; ph < 2.0f * M_PI;
             ph += omega * get_current_meas_period()) {
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(v_alpha, v_beta);

            // Check we meet deadlines after queueing
            this->last_cpu_time = check_timing();
            if (!(this->last_cpu_time < this->control_deadline)) {
                this->error = ERROR_SCAN_MOTOR_TIMING;
                return;
            }
        }
    }
}

void queue_voltage_timings(float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    queue_modulation_timings(mod_alpha, mod_beta);
}

void queue_modulation_timings(float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    this->next_timings[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    this->next_timings[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    this->next_timings[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
}

uint16_t check_timing() {
    TIM_HandleTypeDef *htim = this->motor_timer;
    uint16_t timing = htim->Instance->CNT;
    bool down = htim->Instance->CR1 & TIM_CR1_DIR;
    if (down) {
        uint16_t delta = TIM_1_8_PERIOD_CLOCKS - timing;
        timing = TIM_1_8_PERIOD_CLOCKS + delta;
    }

    if (++(this->timing_log_index) == TIMING_LOG_SIZE) {
        this->timing_log_index = 0;
    }
    this->timing_log[this->timing_log_index] = timing;

    return timing;
}

bool check_deadlines() {
    // Check we meet deadlines after queueing
    this->last_cpu_time = check_timing();
    if (!(this->last_cpu_time < this->control_deadline)) return false;
    return true;
}