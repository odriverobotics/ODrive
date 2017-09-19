#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4
#include <arm_math.h>

#include "motor.h"

#include <main.h>
#include <spi.h>
#include <tim.h>
#include <utils.h>


static uint8_t numMotors = 0;
static Motor* motors[MAX_NUM_MOTORS];

// Need to put this in wherever we're declaring the motor objects
DRV8301_Obj gateDriver0 = {
    .spiHandle = &hspi3,
    // Note: this board has the EN_Gate pin shared!
    .EngpioHandle = EN_GATE_GPIO_Port,
    .EngpioNumber = EN_GATE_Pin,
    .nCSgpioHandle = M0_nCS_GPIO_Port,
    .nCSgpioNumber = M0_nCS_Pin,
    .RxTimeOut = false,
    .enableTimeOut = false
};

DRV8301_Obj gateDriver1 = {
    .spiHandle = &hspi3,
    // Note: this board has the EN_Gate pin shared!
    .EngpioHandle = EN_GATE_GPIO_Port,
    .EngpioNumber = EN_GATE_Pin,
    .nCSgpioHandle = M1_nCS_GPIO_Port,
    .nCSgpioNumber = M1_nCS_Pin,
    .RxTimeOut = false,
    .enableTimeOut = false
};

Current_control_t currentControl0 = {
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
};

Current_control_t currentControl1 = {
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
};

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

Motor::Motor() {
    motorID = numMotors++;
    motors[motorID] = this;
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
    this->motorThread = 0;
    this->thread_ready = false;
    this->enable_control = true;
    this->do_calibration = true;
    this->calibration_ok = false;
    this->motor_timer = (motorID ? &htim8 : &htim1);

    this->next_timings[0] = TIM_1_8_PERIOD_CLOCKS / 2;
    this->next_timings[1] = TIM_1_8_PERIOD_CLOCKS / 2;
    this->next_timings[2] = TIM_1_8_PERIOD_CLOCKS / 2;

    this->control_deadline = TIM_1_8_PERIOD_CLOCKS;
    this->last_cpu_time = 0;

    this->current_meas.phB = 0.0f;
    this->current_meas.phC = 0.0f;

    this->DC_calib.phB = 0.0f;
    this->DC_calib.phC = 0.0f;

    this->shunt_conductance = 1.0f / 0.0005f;  // [S]
    this->phase_current_rev_gain = 0.0f;       // to be set by DRV8301_setup

    this->rotor_mode = ROTOR_MODE_SENSORLESS;

    this->current_control = (motorID ? &currentControl1 : &currentControl0);
    this->gate_driver = (motorID ? gateDriver1 : gateDriver0);
    this->encoder = new Encoder(motorID ? &htim4 : &htim3);
    this->sensorless = new Sensorless();

    this->timing_log_index = 0;
    
    for(int i = 0; i < TIMING_LOG_SIZE; i++)
        this->timing_log[i] = 0;
}

uint8_t Motor::getNumMotors(){
    return numMotors;
}

Motor* Motor::getMotorByID(int nID){
    if(nID >= 0 && nID < numMotors)
        return motors[nID];
    else
        return NULL;
}

void Motor::controlMotorLoop() {
    while (this->enable_control) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
            this->error = ERROR_FOC_MEASUREMENT_TIMEOUT;
            break;
        }
        updateRotor();

        // Position control
        // TODO Decide if we want to use encoder or pll position here
        float vel_des = this->vel_setpoint;
        if (this->control_mode >= CTRL_MODE_POSITION_CONTROL) {
            if (this->rotor_mode == ROTOR_MODE_SENSORLESS) {
                this->error = ERROR_POS_CTRL_DURING_SENSORLESS;
                break;
            }
            float pos_err = this->pos_setpoint - this->encoder->pll_pos;
            vel_des += this->pos_gain * pos_err;
        }

        // Velocity limiting
        vel_des = MACRO_CONSTRAIN(vel_des, -this->vel_limit, this->vel_limit);

        // Velocity control
        float Iq = this->current_setpoint;
        float v_err = vel_des - getPLLVelocity();
        if (this->control_mode >= CTRL_MODE_VELOCITY_CONTROL) {
            Iq += this->vel_gain * v_err;
        }

        // Velocity integral action before limiting
        Iq += this->vel_integrator_current;

        // Apply motor direction correction
        if (this->rotor_mode == ROTOR_MODE_ENCODER ||
            this->rotor_mode == ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS) {
            Iq *= this->encoder->motor_dir;
        }

        // Current limiting
        float Ilim = this->current_control->current_lim;
        bool limited = false;
        if (Iq > Ilim) {
            limited = true;
            Iq = Ilim;
        }
        if (Iq < -Ilim) {
            limited = true;
            Iq = -Ilim;
        }

        // Velocity integrator (behaviour dependent on limiting)
        if (this->control_mode < CTRL_MODE_VELOCITY_CONTROL) {
            // reset integral if not in use
            this->vel_integrator_current = 0.0f;
        } else {
            if (limited) {
                // TODO make decayfactor configurable
                this->vel_integrator_current *= 0.99f;
            } else {
                this->vel_integrator_current +=
                    (this->vel_integrator_gain * getCurrentMeasPeriod()) *
                    v_err;
            }
        }

        // Execute current command
        if (!focCurrent(0.0f, Iq)) {
            break;  // in case of error exit loop, this->error has been set by
                    // focCurrent
        }
    }

    // We are exiting control, reset Ibus, and update brake current
    // TODO update brake current from all motors in 1 func
    // TODO reset this motor Ibus, then call from here
}

bool Motor::calibrateMotor() {
    this->error = ERROR_NO_ERROR;

    // #warning(hardcoded values for SK3-5065-280kv!)
    // float R = 0.0332548246f;
    // float L = 7.97315806e-06f;

    if (!measurePhaseResistance(this->calibration_current, 1.0f))
        return false;
    if (!measurePhaseInductance(-1.0f, 1.0f)) return false;
    if (this->rotor_mode == ROTOR_MODE_ENCODER ||
        this->rotor_mode == ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS) {
        if (!calibrateEncoderOffset(this->calibration_current * this->phase_resistance))
            return false;
    }

    calculateCurrentGains();
    if (!calculatePLLGains()) {
        return false;
    }

    return true;
}

// Calculate current control gains
void Motor::calculateCurrentGains() {
    float current_control_bandwidth = 1000.0f;  // [rad/s]
    this->current_control->p_gain =
        current_control_bandwidth * this->phase_inductance;
    float plant_pole = this->phase_resistance / this->phase_inductance;
    this->current_control->i_gain = plant_pole * this->current_control->p_gain;
}

// Calculate encoder pll gains
bool Motor::calculatePLLGains() {
    float encoder_pll_bandwidth = 1000.0f;  // [rad/s]
    this->encoder->pll_kp = 2.0f * encoder_pll_bandwidth;

    // Check that we don't get problems with discrete time approximation
    if (!(getCurrentMeasPeriod() * this->encoder->pll_kp < 1.0f)) {
        this->error = ERROR_CALIBRATION_TIMING;
        return false;
    }

    // Critically damped
    this->encoder->pll_ki =
        0.25f * (this->encoder->pll_kp * this->encoder->pll_kp);

    // sensorless pll same as encoder (for now)
    this->sensorless->pll_kp = this->encoder->pll_kp;
    this->sensorless->pll_ki = this->encoder->pll_ki;
    return true;
}

// TODO check Ibeta balance to verify good motor connection
bool Motor::measurePhaseResistance(float test_current, float max_voltage) {
    static const float kI = 10.0f;                           //[(V/s)/A]
    int num_test_cycles = 3.0f / getCurrentMeasPeriod();  // Test runs for 3s
    float test_voltage = 0.0f;
    for (int i = 0; i < num_test_cycles; ++i) {
        osEvent evt =
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT);
        if (evt.status != osEventSignal) {
            this->error = ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT;
            return false;
        }
        float Ialpha = -(this->current_meas.phB + this->current_meas.phC);
        test_voltage += (kI * getCurrentMeasPeriod()) * (test_current - Ialpha);
        test_voltage = MACRO_CONSTRAIN(test_voltage, -max_voltage, max_voltage);

        // Test voltage along phase A
        queueVoltageTimings(test_voltage, 0.0f);

        // Check we meet deadlines after queueing
        if (!checkDeadlines()) {
            this->error = ERROR_PHASE_RESISTANCE_TIMING;
            return false;
        }
    }

    // De-energize motor
    queueVoltageTimings(0.0f, 0.0f);

    float R = test_voltage / test_current;
    if (fabs(test_voltage) == fabs(max_voltage) || R < 0.01f || R > 1.0f) {
        this->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        return false;
    }
    this->phase_resistance = R;
    return true;
}

// Melon Refactor - Good
bool Motor::measurePhaseInductance(float voltage_low, float voltage_high) {
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
            queueVoltageTimings(test_voltages[i], 0.0f);

            // Check we meet deadlines after queueing
            if (!checkDeadlines()) {
                this->error = ERROR_PHASE_INDUCTANCE_TIMING;
                return false;
            }
        }
    }

    // De-energize motor
    queueVoltageTimings(0.0f, 0.0f);

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a
    // finite timestep.
    // However, the discretisation in the current control loop inverts the same
    // discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) /
                     (getCurrentMeasPeriod() * (float)num_cycles);
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
bool Motor::calibrateEncoderOffset(float voltage_magnitude) {
    static const float start_lock_duration = 1.0f;
    static const int num_steps = 1024;
    static const float dt_step = 1.0f / 500.0f;
    static const float scan_range = 4.0f * M_PI;
    const float step_size =
        scan_range / (float)num_steps;  // TODO handle const expressions better
                                        // (maybe switch to C++ ?)

    int32_t init_enc_val = (int16_t)this->encoder->encoder_timer->Instance->CNT;
    int32_t encvaluesum = 0;

    // go to encoder zero phase for start_lock_duration to get ready to scan
    for (int i = 0; i < start_lock_duration * getCurrentMeasHZ(); ++i) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT)
                .status != osEventSignal) {
            this->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
            return false;
        }
        queueVoltageTimings(voltage_magnitude, 0.0f);
    }
    // scan forwards
    for (float ph = -scan_range / 2.0f; ph < scan_range / 2.0f; ph += step_size) {
        for (int i = 0; i < dt_step * (float)getCurrentMeasHZ(); ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT)
                    .status != osEventSignal) {
                this->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queueVoltageTimings(v_alpha, v_beta);
        }
        encvaluesum += (int16_t)this->encoder->encoder_timer->Instance->CNT;
    }
    // check direction
    if ((int16_t)this->encoder->encoder_timer->Instance->CNT > init_enc_val + 8) {
        // motor same dir as encoder
        this->encoder->motor_dir = 1;
    } else if ((int16_t)this->encoder->encoder_timer->Instance->CNT <
               init_enc_val - 8) {
        // motor opposite dir as encoder
        this->encoder->motor_dir = -1;
    } else {
        // Encoder response error
        this->error = ERROR_ENCODER_RESPONSE;
        return false;
    }
    // scan backwards
    for (float ph = scan_range / 2.0f; ph > -scan_range / 2.0f; ph -= step_size) {
        for (int i = 0; i < dt_step * (float)getCurrentMeasHZ(); ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT)
                    .status != osEventSignal) {
                this->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queueVoltageTimings(v_alpha, v_beta);
        }
        encvaluesum += (int16_t)this->encoder->encoder_timer->Instance->CNT;
    }

    int offset = encvaluesum / (num_steps * 2);
    this->encoder->encoder_offset = offset;
    return true;
}

void Motor::scanMotorLoop(float omega, float voltage_magnitude) {
    for (;;) {
        for (float ph = 0.0f; ph < 2.0f * M_PI;
             ph += omega * getCurrentMeasPeriod()) {
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queueVoltageTimings(v_alpha, v_beta);

            // Check we meet deadlines after queueing
            this->last_cpu_time = checkTiming();
            if (!(this->last_cpu_time < this->control_deadline)) {
                this->error = ERROR_SCAN_MOTOR_TIMING;
                return;
            }
        }
    }
}

void Motor::queueVoltageTimings(float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    queueModulationTimings(mod_alpha, mod_beta);
}

void Motor::queueModulationTimings(float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    this->next_timings[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    this->next_timings[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    this->next_timings[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
}

uint16_t Motor::checkTiming() {
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

bool Motor::checkDeadlines() {
    // Check we meet deadlines after queueing
    this->last_cpu_time = checkTiming();
    if (!(this->last_cpu_time < this->control_deadline)) return false;
    return true;
}

//--------------------------------
// Main motor control
//--------------------------------`

void Motor::updateRotor() {
    switch (this->rotor_mode) {
        case ROTOR_MODE_ENCODER:
            updateEncoder();
            break;
        case ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS:
            updateEncoder();  // If we're testing, we want both encoder
                                    // and sensorless
        case ROTOR_MODE_SENSORLESS:
            updateSensorless();
            break;
        default:
            // TODO error handling
            break;
    }
}

void Motor::updateEncoder() {
    // For convenience
    Encoder *encoder = this->encoder;

    int16_t delta_enc = (int16_t)encoder->encoder_timer->Instance->CNT -
                        (int16_t)encoder->encoder_state;
    encoder->encoder_state += (int32_t)delta_enc;

    // compute electrical phase
    int corrected_enc = encoder->encoder_state % ENCODER_CPR;
    corrected_enc -= encoder->encoder_offset;
    corrected_enc *= encoder->motor_dir;
    float ph = elec_rad_per_enc * (float)corrected_enc;
    // ph = fmodf(ph, 2*M_PI);
    encoder->phase = wrap_pm_pi(ph);

    // run pll (for now pll is in units of encoder counts)
    // TODO pll_pos runs out of precision very quickly here! Perhaps decompose
    // into integer and fractional part?
    // Predict current pos
    encoder->pll_pos += getCurrentMeasPeriod() * encoder->pll_vel;
    // discrete phase detector
    float delta_pos =
        (float)(encoder->encoder_state - (int32_t)floorf(encoder->pll_pos));
    // pll feedback
    encoder->pll_pos += getCurrentMeasPeriod() * encoder->pll_kp * delta_pos;
    encoder->pll_vel += getCurrentMeasPeriod() * encoder->pll_ki * delta_pos;
}

void Motor::updateSensorless() {
    // Algorithm based on paper: Sensorless Control of Surface-Mount
    // Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer
    // http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
    // In particular, equation 8 (and by extension eqn 4 and 6).

    // The V_alpha_beta applied immedietly prior to the current measurement
    // associated with this cycle
    // is the one computed two cycles ago. To get the correct measurement, it
    // was stored twice:
    // once by final_v_alpha/final_v_beta in the current control reporting, and
    // once by V_alpha_beta_memory.

    // for convenience
    Sensorless *sensorless = this->sensorless;

    // Clarke transform
    float I_alpha_beta[2] = {
        -this->current_meas.phB - this->current_meas.phC,
        ONE_BY_SQRT3 * (this->current_meas.phB - this->current_meas.phC)};

    // alpha-beta vector operations
    float eta[2];
    for (int i = 0; i <= 1; ++i) {
        // y is the total flux-driving voltage (see paper eqn 4)
        float y = -this->phase_resistance * I_alpha_beta[i] +
                  sensorless->V_alpha_beta_memory[i];
        // flux dynamics (prediction)
        float x_dot = y;
        // integrate prediction to current timestep
        sensorless->flux_state[i] += x_dot * getCurrentMeasPeriod();

        // eta is the estimated permanent magnet flux (see paper eqn 6)
        eta[i] = sensorless->flux_state[i] -
                 this->phase_inductance * I_alpha_beta[i];
    }

    // Non-linear observer (see paper eqn 8):
    float pm_flux_sqr =
        sensorless->pm_flux_linkage * sensorless->pm_flux_linkage;
    float est_pm_flux_sqr = eta[0] * eta[0] + eta[1] * eta[1];
    float bandwidth_factor =
        1.0f / (sensorless->pm_flux_linkage * sensorless->pm_flux_linkage);
    float eta_factor = 0.5f * (sensorless->observer_gain * bandwidth_factor) *
                       (pm_flux_sqr - est_pm_flux_sqr);

    static float eta_factor_avg_test = 0.0f;
    eta_factor_avg_test += 0.001f * (eta_factor - eta_factor_avg_test);

    // alpha-beta vector operations
    for (int i = 0; i <= 1; ++i) {
        // add observer action to flux estimate dynamics
        float x_dot = eta_factor * eta[i];
        // convert action to discrete-time
        sensorless->flux_state[i] += x_dot * getCurrentMeasPeriod();
        // update new eta
        eta[i] = sensorless->flux_state[i] -
                 this->phase_inductance * I_alpha_beta[i];
    }

    // Flux state estimation done, store V_alpha_beta for next timestep
    sensorless->V_alpha_beta_memory[0] = this->current_control->final_v_alpha;
    sensorless->V_alpha_beta_memory[1] = this->current_control->final_v_beta;

    // PLL
    // predict PLL phase with velocity
    sensorless->pll_pos = wrap_pm_pi(
        sensorless->pll_pos + getCurrentMeasPeriod() * sensorless->pll_vel);
    // update PLL phase with observer permanent magnet phase
    sensorless->phase = fast_atan2(eta[1], eta[0]);
    float delta_phase = wrap_pm_pi(sensorless->phase - sensorless->pll_pos);
    sensorless->pll_pos = wrap_pm_pi(sensorless->pll_pos +
                                     getCurrentMeasPeriod() *
                                         sensorless->pll_kp * delta_phase);
    // update PLL velocity
    sensorless->pll_vel +=
        getCurrentMeasPeriod() * sensorless->pll_ki * delta_phase;

    // TODO TEMP TEST HACK
    // static int trigger_ctr = 0;
    // if (++trigger_ctr >= 3*current_meas_hz) {
    //     trigger_ctr = 0;

    //     //Change to sensorless units
    //     this->vel_gain = 15.0f / 200.0f;
    //     this->vel_setpoint = 800.0f * this->encoder->motor_dir;

    //     //Change mode
    //     this->rotor_mode = ROTOR_MODE_SENSORLESS;
    // }
}

float Motor::getRotorPhase() {
    switch (this->rotor_mode) {
        case ROTOR_MODE_ENCODER:
        case ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS:
            return this->encoder->phase;
            break;
        case ROTOR_MODE_SENSORLESS:
            return this->sensorless->phase;
            break;
        default:
            // TODO error handling
            return 0.0f;
            break;
    }
}

float Motor::getPLLVelocity() {
    switch (this->rotor_mode) {
        case ROTOR_MODE_ENCODER:
        case ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS:
            return this->encoder->pll_vel;
            break;
        case ROTOR_MODE_SENSORLESS:
            return this->sensorless->pll_vel;
            break;
        default:
            // TODO error handling
            return 0.0f;
            break;
    }
}

bool Motor::spinUpTimestep(float phase, float I_mag) {
    // wait for new timestep
    if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
        this->error = ERROR_SPIN_UP_TIMEOUT;
        return false;
    }
    // run estimator
    updateRotor();
    // override the phase during spinup
    this->sensorless->phase = phase;
    // run current control (with the phase override)
    focCurrent(I_mag, 0.0f);

    return true;
}

bool Motor::spinUpSensorless() {
    static const float ramp_up_time = 0.4f;
    static const float ramp_up_distance = 4 * M_PI;
    float ramp_step = getCurrentMeasPeriod() / ramp_up_time;

    float phase = 0.0f;
    float vel = ramp_up_distance / ramp_up_time;
    float I_mag = 0.0f;

    // spiral up current
    for (float x = 0.0f; x < 1.0f; x += ramp_step) {
        phase = wrap_pm_pi(ramp_up_distance * x);
        I_mag = this->sensorless->spin_up_current * x;
        if (!spinUpTimestep(phase, I_mag)) return false;
    }

    // accelerate
    while (vel < this->sensorless->spin_up_target_vel) {
        vel +=
            this->sensorless->spin_up_acceleration * getCurrentMeasPeriod();
        phase = wrap_pm_pi(phase + vel * getCurrentMeasPeriod());
        if (!spinUpTimestep(phase, this->sensorless->spin_up_current))
            return false;
    }

    // // test keep spinning
    // while (true) {
    //     phase = wrap_pm_pi(phase + vel * getCurrentMeasPeriod());
    //     if(!spinUpTimestep(phase, this->sensorless->spin_up_current))
    //         return false;
    // }

    return true;

    // TODO: check pll vel (abs ratio, 0.8)
}

void Motor::updateBrakeCurrent(float brake_current) {
    if (brake_current < 0.0f) brake_current = 0.0f;
    float brake_duty = brake_current * brake_resistance / vbus_voltage;

    // Duty limit at 90% to allow bootstrap caps to charge
    if (brake_duty > 0.9f) brake_duty = 0.9f;
    int high_on = TIM_APB1_PERIOD_CLOCKS * (1.0f - brake_duty);
    int low_off = high_on - TIM_APB1_DEADTIME_CLOCKS;
    if (low_off < 0) low_off = 0;

    // Safe update of low and high side timings
    // To avoid race condition, first reset timings to safe state
    // ch3 is low side, ch4 is high side
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    htim2.Instance->CCR3 = low_off;
    htim2.Instance->CCR4 = high_on;
}

bool Motor::focCurrent(float Id_des, float Iq_des) {
    Current_control_t *ictrl = this->current_control;

    // Clarke transform
    float Ialpha = -this->current_meas.phB - this->current_meas.phC;
    float Ibeta =
        ONE_BY_SQRT3 * (this->current_meas.phB - this->current_meas.phC);

    // Park transform
    float phase = getRotorPhase();
    float c = arm_cos_f32(phase);
    float s = arm_sin_f32(phase);
    float Id = c * Ialpha + s * Ibeta;
    float Iq = c * Ibeta - s * Ialpha;

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL
    // tau)
    // Apply PI control
    float Vd = ictrl->v_current_control_integral_d + Ierr_d * ictrl->p_gain;
    float Vq = ictrl->v_current_control_integral_q + Ierr_q * ictrl->p_gain;

    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    // Vector modulation saturation, lock integrator if saturated
    // TODO make maximum modulation configurable
    float mod_scalefactor =
        0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // TODO make decayfactor configurable
        ictrl->v_current_control_integral_d *= 0.99f;
        ictrl->v_current_control_integral_q *= 0.99f;
    } else {
        ictrl->v_current_control_integral_d +=
            Ierr_d * (ictrl->i_gain * getCurrentMeasPeriod());
        ictrl->v_current_control_integral_q +=
            Ierr_q * (ictrl->i_gain * getCurrentMeasPeriod());
    }

    // Compute estimated bus current
    ictrl->Ibus = mod_d * Id + mod_q * Iq;

    // If this is last motor, update brake resistor duty
    // if (motor == getMotorByID(numMotors-1) {
    // Above check doesn't work if last motor is executing voltage control
    // TODO trigger this update in controlMotorLoop instead,
    // and make voltage control a control mode in it.
    float Ibus_sum = 0.0f;
    for (int i = 0; i < getNumMotors(); ++i) {
        Ibus_sum += (*getMotorByID(i)).current_control->Ibus;
    }
    // Note: function will clip negative values to 0.0f
    updateBrakeCurrent(-Ibus_sum);
    // }

    // Inverse park transform
    float mod_alpha = c * mod_d - s * mod_q;
    float mod_beta = c * mod_q + s * mod_d;

    // Report final applied voltage in stationary frame (for sensorles
    // estimator)
    ictrl->final_v_alpha = mod_to_V * mod_alpha;
    ictrl->final_v_beta = mod_to_V * mod_beta;

    // Apply SVM
    queueModulationTimings(mod_alpha, mod_beta);

    // Check we meet deadlines after queueing
    this->last_cpu_time = checkTiming();
    if (!(this->last_cpu_time < this->control_deadline)) {
        this->error = ERROR_FOC_TIMING;
        return false;
    }
    return true;
}


//--------------------------------
// Utility
//--------------------------------

float Motor::phaseCurrentFromADCVal(uint32_t ADCValue) {
    int adcval_bal = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * this->phase_current_rev_gain;
    float current = shunt_volt * this->shunt_conductance;
    return current;
}