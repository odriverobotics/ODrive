/* Includes ------------------------------------------------------------------*/

// Because of broken cmsis_os.h, we need to include arm_math first,
// otherwise chip specific defines are ommited
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4
#include <arm_math.h>

#include <low_level.h>

#include <cmsis_os.h>
#include <math.h>
#include <stdlib.h>

#include <adc.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <utils.h>

/* Private defines -----------------------------------------------------------*/

// #define DEBUG_PRINT

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

/* Private constant data -----------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
// Command Handling

// Utility

static void global_fault(int error);
static float phase_current_from_adcval(Motor_t *motor, uint32_t ADCValue);
// Initalisation
static void DRV8301_setup(Motor_t *motor);
static void start_adc_pwm();
static void start_pwm(TIM_HandleTypeDef *htim);
static void sync_timers(TIM_HandleTypeDef *htim_a, TIM_HandleTypeDef *htim_b,
                        uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset);
// IRQ Callbacks (are all public)
// Measurement and calibrationa

// Test functions

static void FOC_voltage_loop(Motor_t *motor, float v_d, float v_q);
// Main motor control
static void update_rotor(Motor_t *motor);
static void update_sensorless(Motor_t *motor);
static void update_encoder(Motor_t *motor);
static float get_rotor_phase(Motor_t *motor);
static float get_pll_vel(Motor_t *motor);
static bool spin_up_sensorless(Motor_t *motor);
static void update_brake_current(float brake_current);

static bool FOC_current(Motor_t *motor, float Id_des, float Iq_des);
// Motor thread (is public)

/* Function implementations --------------------------------------------------*/

//--------------------------------
// Utility
//--------------------------------

static void global_fault(int error) {
    // Disable motors NOW!
    for (int i = 0; i < num_motors; ++i) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motors[i].motor_timer);
    }
    // Set fault codes, etc.
    for (int i = 0; i < num_motors; ++i) {
        motors[i].error = error;
        motors[i].enable_control = false;
        motors[i].calibration_ok = false;
    }
    // disable brake resistor
    update_brake_current(0.0f);
}

static float phase_current_from_adcval(Motor_t *motor, uint32_t ADCValue) {
    int adcval_bal = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * motor->phase_current_rev_gain;
    float current = shunt_volt * motor->shunt_conductance;
    return current;
}

//--------------------------------
// Initalisation
//--------------------------------

// Initalises the low level motor control and then starts the motor control
// threads
void init_motor_control() {
    // Init gate drivers
    DRV8301_setup(&motors[0]);
    DRV8301_setup(&motors[1]);

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    // Start Encoders
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // Wait for current sense calibration to converge
    // TODO make timing a function of calibration filter tau
    osDelay(1500);
}

// Set up the gate drivers
static void DRV8301_setup(Motor_t *motor) {
    DRV8301_Obj *gate_driver = &motor->gate_driver;
    DRV_SPI_8301_Vars_t *local_regs = &motor->gate_driver_regs;

    DRV8301_enable(gate_driver);
    DRV8301_setupSpi(gate_driver, local_regs);

    // TODO we can use reporting only if we actually wire up the nOCTW pin
    local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    local_regs->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_40VpV;

    switch (local_regs->Ctrl_Reg_2.GAIN) {
        case DRV8301_ShuntAmpGain_10VpV:
            motor->phase_current_rev_gain = 1.0f / 10.0f;
            break;
        case DRV8301_ShuntAmpGain_20VpV:
            motor->phase_current_rev_gain = 1.0f / 20.0f;
            break;
        case DRV8301_ShuntAmpGain_40VpV:
            motor->phase_current_rev_gain = 1.0f / 40.0f;
            break;
        case DRV8301_ShuntAmpGain_80VpV:
            motor->phase_current_rev_gain = 1.0f / 80.0f;
            break;
    }

    local_regs->SndCmd = true;
    DRV8301_writeData(gate_driver, local_regs);
    local_regs->RcvCmd = true;
    DRV8301_readData(gate_driver, local_regs);
}

static void start_adc_pwm() {
    // Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    // Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_EOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_EOC);

    // Ensure that debug halting of the core doesn't leave the motor PWM running
    __HAL_DBGMCU_FREEZE_TIM1();
    __HAL_DBGMCU_FREEZE_TIM8();

    start_pwm(&htim1);
    start_pwm(&htim8);
    // TODO: explain why this offset
    sync_timers(&htim1, &htim8, TIM_CLOCKSOURCE_ITR0,
                TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128);

    // Motor output starts in the disabled state
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    // Start brake resistor PWM in floating output configuration
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

static void start_pwm(TIM_HandleTypeDef *htim) {
    // Init PWM
    int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
    htim->Instance->CCR1 = half_load;
    htim->Instance->CCR2 = half_load;
    htim->Instance->CCR3 = half_load;

    // This hardware obfustication layer really is getting on my nerves
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);

    htim->Instance->CCR4 = 1;
    HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_4);
}

static void sync_timers(TIM_HandleTypeDef *htim_a, TIM_HandleTypeDef *htim_b,
                        uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset) {
    // Store intial timer configs
    uint16_t MOE_store_a = htim_a->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t MOE_store_b = htim_b->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t CR2_store = htim_a->Instance->CR2;
    uint16_t SMCR_store = htim_b->Instance->SMCR;
    // Turn off output
    htim_a->Instance->BDTR &= ~(TIM_BDTR_MOE);
    htim_b->Instance->BDTR &= ~(TIM_BDTR_MOE);
    // Disable both timer counters
    htim_a->Instance->CR1 &= ~TIM_CR1_CEN;
    htim_b->Instance->CR1 &= ~TIM_CR1_CEN;
    // Set first timer to send TRGO on counter enable
    htim_a->Instance->CR2 &= ~TIM_CR2_MMS;
    htim_a->Instance->CR2 |= TIM_TRGO_ENABLE;
    // Set Trigger Source of second timer to the TRGO of the first timer
    htim_b->Instance->SMCR &= ~TIM_SMCR_TS;
    htim_b->Instance->SMCR |= TIM_CLOCKSOURCE_ITRx;
    // Set 2nd timer to start on trigger
    htim_b->Instance->SMCR &= ~TIM_SMCR_SMS;
    htim_b->Instance->SMCR |= TIM_SLAVEMODE_TRIGGER;
    // Dir bit is read only in center aligned mode, so we clear the mode for now
    uint16_t CMS_store_a = htim_a->Instance->CR1 & TIM_CR1_CMS;
    uint16_t CMS_store_b = htim_b->Instance->CR1 & TIM_CR1_CMS;
    htim_a->Instance->CR1 &= ~TIM_CR1_CMS;
    htim_b->Instance->CR1 &= ~TIM_CR1_CMS;
    // Set both timers to up-counting state
    htim_a->Instance->CR1 &= ~TIM_CR1_DIR;
    htim_b->Instance->CR1 &= ~TIM_CR1_DIR;
    // Restore center aligned mode
    htim_a->Instance->CR1 |= CMS_store_a;
    htim_b->Instance->CR1 |= CMS_store_b;
    // set counter offset
    htim_a->Instance->CNT = count_offset;
    htim_b->Instance->CNT = 0;
    // Start Timer a
    htim_a->Instance->CR1 |= (TIM_CR1_CEN);
    // Restore timer configs
    htim_a->Instance->CR2 = CR2_store;
    htim_b->Instance->SMCR = SMCR_store;
    // restore output
    htim_a->Instance->BDTR |= MOE_store_a;
    htim_b->Instance->BDTR |= MOE_store_b;
}

//--------------------------------
// IRQ Callbacks
//--------------------------------

// This is the callback from the ADC that we expect after the PWM has triggered
// an ADC conversion.
// TODO: Document how the phasing is done, link to timing diagram
void pwm_trig_adc_cb(ADC_HandleTypeDef *hadc, bool injected) {
#define calib_tau 0.2f  //@TOTO make more easily configurable
    float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

    // Ensure ADCs are expected ones to simplify the logic below
    if (!(hadc == &hadc2 || hadc == &hadc3)) {
        global_fault(ERROR_ADC_FAILED);
        return;
    };

    // Motor 0 is on Timer 1, which triggers ADC 2 and 3 on an injected
    // conversion
    // Motor 1 is on Timer 8, which triggers ADC 2 and 3 on a regular conversion
    // If the corresponding timer is counting up, we just sampled in SVM vector
    // 0, i.e. real current
    // If we are counting down, we just sampled in SVM vector 7, with zero
    // current
    Motor_t *motor = injected ? &motors[0] : &motors[1];
    bool counting_down = motor->motor_timer->Instance->CR1 & TIM_CR1_DIR;

    bool current_meas_not_DC_CAL;
    if (motor == &motors[1] && counting_down) {
        // We are measuring M1 DC_CAL here
        current_meas_not_DC_CAL = false;
        // Load next timings for M0 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[0].motor_timer->Instance->CCR1 = motors[0].next_timings[0];
            motors[0].motor_timer->Instance->CCR2 = motors[0].next_timings[1];
            motors[0].motor_timer->Instance->CCR3 = motors[0].next_timings[2];
        }
        // Check the timing of the sequencing
        check_timing(motor);
    } else if (motor == &motors[0] && !counting_down) {
        // We are measuring M0 current here
        current_meas_not_DC_CAL = true;
        // Load next timings for M1 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[1].motor_timer->Instance->CCR1 = motors[1].next_timings[0];
            motors[1].motor_timer->Instance->CCR2 = motors[1].next_timings[1];
            motors[1].motor_timer->Instance->CCR3 = motors[1].next_timings[2];
        }
        // Check the timing of the sequencing
        check_timing(motor);
    } else if (motor == &motors[1] && !counting_down) {
        // We are measuring M1 current here
        current_meas_not_DC_CAL = true;
        // Check the timing of the sequencing
        check_timing(motor);
    } else if (motor == &motors[0] && counting_down) {
        // We are measuring M0 DC_CAL here
        current_meas_not_DC_CAL = false;
        // Check the timing of the sequencing
        check_timing(motor);
    } else {
        global_fault(ERROR_PWM_SRC_FAIL);
        return;
    }

    uint32_t ADCValue;
    if (injected) {
        ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    } else {
        ADCValue = HAL_ADC_GetValue(hadc);
    }
    float current = phase_current_from_adcval(motor, ADCValue);

    if (current_meas_not_DC_CAL) {
        // ADC2 and ADC3 record the phB and phC currents concurrently,
        // and their interrupts should arrive on the same clock cycle.
        // We dispatch the callbacks in order, so ADC2 will always be processed
        // before ADC3.
        // Therefore we store the value from ADC2 and signal the thread that the
        // measurement is ready when we recieve the ADC3 measurement

        // return or continue
        if (hadc == &hadc2) {
            motor->current_meas.phB = current - motor->DC_calib.phB;
            return;
        } else {
            motor->current_meas.phC = current - motor->DC_calib.phC;
        }
        // Trigger motor thread
        if (motor->thread_ready)
            osSignalSet(motor->motor_thread, M_SIGNAL_PH_CURRENT_MEAS);
    } else {
        // DC_CAL measurement
        if (hadc == &hadc2) {
            motor->DC_calib.phB +=
                (current - motor->DC_calib.phB) * calib_filter_k;
        } else {
            motor->DC_calib.phC +=
                (current - motor->DC_calib.phC) * calib_filter_k;
        }
    }
}

//--------------------------------
// Test functions
//--------------------------------

// TODO integrate as mode in main control loop
static void FOC_voltage_loop(Motor_t *motor, float v_d, float v_q) {
    for (;;) {
        osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
        update_rotor(motor);

        float phase = get_rotor_phase(motor);
        float c = arm_cos_f32(phase);
        float s = arm_sin_f32(phase);
        float v_alpha = c * v_d - s * v_q;
        float v_beta = c * v_q + s * v_d;
        queue_voltage_timings(motor, v_alpha, v_beta);

        // Check we meet deadlines after queueing
        motor->last_cpu_time = check_timing(motor);
        if (!(motor->last_cpu_time < motor->control_deadline)) {
            motor->error = ERROR_FOC_VOLTAGE_TIMING;
            return;
        }
    }
}

//--------------------------------
// Main motor control
//--------------------------------`

static void update_rotor(Motor_t *motor) {
    switch (motor->rotor_mode) {
        case ROTOR_MODE_ENCODER:
            update_encoder(motor);
            break;
        case ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS:
            update_encoder(motor);  // If we're testing, we want both encoder
                                    // and sensorless
        case ROTOR_MODE_SENSORLESS:
            update_sensorless(motor);
            break;
        default:
            // TODO error handling
            break;
    }
}

void update_encoder(Motor_t *motor) {
    // For convenience
    Encoder_t *encoder = &motor->encoder;

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
    encoder->pll_pos += get_current_meas_period() * encoder->pll_vel;
    // discrete phase detector
    float delta_pos =
        (float)(encoder->encoder_state - (int32_t)floorf(encoder->pll_pos));
    // pll feedback
    encoder->pll_pos += get_current_meas_period() * encoder->pll_kp * delta_pos;
    encoder->pll_vel += get_current_meas_period() * encoder->pll_ki * delta_pos;
}

void update_sensorless(Motor_t *motor) {
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
    ODrive_Sensorless_t *sensorless = &motor->sensorless;

    // Clarke transform
    float I_alpha_beta[2] = {
        -motor->current_meas.phB - motor->current_meas.phC,
        ONE_BY_SQRT3 * (motor->current_meas.phB - motor->current_meas.phC)};

    // alpha-beta vector operations
    float eta[2];
    for (int i = 0; i <= 1; ++i) {
        // y is the total flux-driving voltage (see paper eqn 4)
        float y = -motor->phase_resistance * I_alpha_beta[i] +
                  sensorless->V_alpha_beta_memory[i];
        // flux dynamics (prediction)
        float x_dot = y;
        // integrate prediction to current timestep
        sensorless->flux_state[i] += x_dot * get_current_meas_period();

        // eta is the estimated permanent magnet flux (see paper eqn 6)
        eta[i] = sensorless->flux_state[i] -
                 motor->phase_inductance * I_alpha_beta[i];
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
        sensorless->flux_state[i] += x_dot * get_current_meas_period();
        // update new eta
        eta[i] = sensorless->flux_state[i] -
                 motor->phase_inductance * I_alpha_beta[i];
    }

    // Flux state estimation done, store V_alpha_beta for next timestep
    sensorless->V_alpha_beta_memory[0] = motor->current_control.final_v_alpha;
    sensorless->V_alpha_beta_memory[1] = motor->current_control.final_v_beta;

    // PLL
    // predict PLL phase with velocity
    sensorless->pll_pos = wrap_pm_pi(
        sensorless->pll_pos + get_current_meas_period() * sensorless->pll_vel);
    // update PLL phase with observer permanent magnet phase
    sensorless->phase = fast_atan2(eta[1], eta[0]);
    float delta_phase = wrap_pm_pi(sensorless->phase - sensorless->pll_pos);
    sensorless->pll_pos = wrap_pm_pi(sensorless->pll_pos +
                                     get_current_meas_period() *
                                         sensorless->pll_kp * delta_phase);
    // update PLL velocity
    sensorless->pll_vel +=
        get_current_meas_period() * sensorless->pll_ki * delta_phase;

    // TODO TEMP TEST HACK
    // static int trigger_ctr = 0;
    // if (++trigger_ctr >= 3*current_meas_hz) {
    //     trigger_ctr = 0;

    //     //Change to sensorless units
    //     motor->vel_gain = 15.0f / 200.0f;
    //     motor->vel_setpoint = 800.0f * motor->encoder.motor_dir;

    //     //Change mode
    //     motor->rotor_mode = ROTOR_MODE_SENSORLESS;
    // }
}

static float get_rotor_phase(Motor_t *motor) {
    switch (motor->rotor_mode) {
        case ROTOR_MODE_ENCODER:
        case ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS:
            return motor->encoder.phase;
            break;
        case ROTOR_MODE_SENSORLESS:
            return motor->sensorless.phase;
            break;
        default:
            // TODO error handling
            return 0.0f;
            break;
    }
}

static float get_pll_vel(Motor_t *motor) {
    switch (motor->rotor_mode) {
        case ROTOR_MODE_ENCODER:
        case ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS:
            return motor->encoder.pll_vel;
            break;
        case ROTOR_MODE_SENSORLESS:
            return motor->sensorless.pll_vel;
            break;
        default:
            // TODO error handling
            return 0.0f;
            break;
    }
}

static bool spin_up_timestep(Motor_t *motor, float phase, float I_mag) {
    // wait for new timestep
    if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT)
            .status != osEventSignal) {
        motor->error = ERROR_SPIN_UP_TIMEOUT;
        return false;
    }
    // run estimator
    update_rotor(motor);
    // override the phase during spinup
    motor->sensorless.phase = phase;
    // run current control (with the phase override)
    FOC_current(motor, I_mag, 0.0f);

    return true;
}

static bool spin_up_sensorless(Motor_t *motor) {
    static const float ramp_up_time = 0.4f;
    static const float ramp_up_distance = 4 * M_PI;
    float ramp_step = get_current_meas_period() / ramp_up_time;

    float phase = 0.0f;
    float vel = ramp_up_distance / ramp_up_time;
    float I_mag = 0.0f;

    // spiral up current
    for (float x = 0.0f; x < 1.0f; x += ramp_step) {
        phase = wrap_pm_pi(ramp_up_distance * x);
        I_mag = motor->sensorless.spin_up_current * x;
        if (!spin_up_timestep(motor, phase, I_mag)) return false;
    }

    // accelerate
    while (vel < motor->sensorless.spin_up_target_vel) {
        vel +=
            motor->sensorless.spin_up_acceleration * get_current_meas_period();
        phase = wrap_pm_pi(phase + vel * get_current_meas_period());
        if (!spin_up_timestep(motor, phase, motor->sensorless.spin_up_current))
            return false;
    }

    // // test keep spinning
    // while (true) {
    //     phase = wrap_pm_pi(phase + vel * get_current_meas_period());
    //     if(!spin_up_timestep(motor, phase,
    //     motor->sensorless.spin_up_current))
    //         return false;
    // }

    return true;

    // TODO: check pll vel (abs ratio, 0.8)
}

static void update_brake_current(float brake_current) {
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

void control_motor_loop(Motor_t &motor) {
    while (motor.enable_control) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
            motor.error = ERROR_FOC_MEASUREMENT_TIMEOUT;
            break;
        }
        update_rotor(motor);

        // Position control
        // TODO Decide if we want to use encoder or pll position here
        float vel_des = motor.vel_setpoint;
        if (motor.control_mode >= CTRL_MODE_POSITION_CONTROL) {
            if (motor.rotor_mode == ROTOR_MODE_SENSORLESS) {
                motor.error = ERROR_POS_CTRL_DURING_SENSORLESS;
                break;
            }
            float pos_err = motor.pos_setpoint - motor.encoder.pll_pos;
            vel_des += motor.pos_gain * pos_err;
        }

        // Velocity limiting
        vel_des = MACRO_CONSTRAIN(vel_des, -motor.vel_limit, motor.vel_limit);

        // Velocity control
        float Iq = motor.current_setpoint;
        float v_err = vel_des - get_pll_vel(motor);
        if (motor.control_mode >= CTRL_MODE_VELOCITY_CONTROL) {
            Iq += motor.vel_gain * v_err;
        }

        // Velocity integral action before limiting
        Iq += motor.vel_integrator_current;

        // Apply motor direction correction
        if (motor.rotor_mode == ROTOR_MODE_ENCODER ||
            motor.rotor_mode == ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS) {
            Iq *= motor.encoder.motor_dir;
        }

        // Current limiting
        float Ilim = motor.current_control.current_lim;
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
        if (motor.control_mode < CTRL_MODE_VELOCITY_CONTROL) {
            // reset integral if not in use
            motor.vel_integrator_current = 0.0f;
        } else {
            if (limited) {
                // TODO make decayfactor configurable
                motor.vel_integrator_current *= 0.99f;
            } else {
                motor.vel_integrator_current +=
                    (motor.vel_integrator_gain * get_current_meas_period()) *
                    v_err;
            }
        }

        // Execute current command
        if (!FOC_current(motor, 0.0f, Iq)) {
            break;  // in case of error exit loop, motor.error has been set by
                    // FOC_current
        }
    }

    // We are exiting control, reset Ibus, and update brake current
    // TODO update brake current from all motors in 1 func
    // TODO reset this motor Ibus, then call from here
}

static bool FOC_current(Motor_t *motor, float Id_des, float Iq_des) {
    Current_control_t *ictrl = &motor->current_control;

    // Clarke transform
    float Ialpha = -motor->current_meas.phB - motor->current_meas.phC;
    float Ibeta =
        ONE_BY_SQRT3 * (motor->current_meas.phB - motor->current_meas.phC);

    // Park transform
    float phase = get_rotor_phase(motor);
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
            Ierr_d * (ictrl->i_gain * get_current_meas_period());
        ictrl->v_current_control_integral_q +=
            Ierr_q * (ictrl->i_gain * get_current_meas_period());
    }

    // Compute estimated bus current
    ictrl->Ibus = mod_d * Id + mod_q * Iq;

    // If this is last motor, update brake resistor duty
    // if (motor == &motors[num_motors-1]) {
    // Above check doesn't work if last motor is executing voltage control
    // TODO trigger this update in control_motor_loop instead,
    // and make voltage control a control mode in it.
    float Ibus_sum = 0.0f;
    for (int i = 0; i < num_motors; ++i) {
        Ibus_sum += motors[i].current_control.Ibus;
    }
    // Note: function will clip negative values to 0.0f
    update_brake_current(-Ibus_sum);
    // }

    // Inverse park transform
    float mod_alpha = c * mod_d - s * mod_q;
    float mod_beta = c * mod_q + s * mod_d;

    // Report final applied voltage in stationary frame (for sensorles
    // estimator)
    ictrl->final_v_alpha = mod_to_V * mod_alpha;
    ictrl->final_v_beta = mod_to_V * mod_beta;

    // Apply SVM
    queue_modulation_timings(motor, mod_alpha, mod_beta);

    // Check we meet deadlines after queueing
    motor->last_cpu_time = check_timing(motor);
    if (!(motor->last_cpu_time < motor->control_deadline)) {
        motor->error = ERROR_FOC_TIMING;
        return false;
    }
    return true;
}

//--------------------------------
// Motor thread
//--------------------------------

void motor_thread(int motorID) {
    Motor motor = new Motor(motorID);  // Not a problem as long as we don't deallocate it
    motor.motor_thread = osThreadGetId();
    motor.thread_ready = true;

    for (;;) {
        if (motor.do_calibration) {
            __HAL_TIM_MOE_ENABLE(motor.motor_timer);                   // enable pwm outputs
            motor.calibrateMotor();
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor.motor_timer);  // disables pwm outputs
            motor.do_calibration = false;
        }

        if (motor.calibration_ok && motor.enable_control) {
            motor.enable_step_dir = true;
            __HAL_TIM_MOE_ENABLE(motor.motor_timer);

            bool spin_up_ok = true;
            if (motor.rotor_mode == ROTOR_MODE_SENSORLESS)
                spin_up_ok = spin_up_sensorless(motor);
            if (spin_up_ok)
                control_motor_loop(motor);  // This doesn't return until motor.enable_control is false or there's an error.

            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor.motor_timer);
            motor.enable_step_dir = false;

            if (motor.enable_control) {  // if control is still enabled, we exited because of error
                motor.calibration_ok = false;
                motor.enable_control = false;
            }
        }

        queue_voltage_timings(motor, 0.0f, 0.0f);
        osDelay(100);
    }
    motor.thread_ready = false;
}