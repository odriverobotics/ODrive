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

#include "error.h"
#include "motor.h"

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



// Initalisation
static void DRV8301_setup(Motor &motor);
static void start_adc_pwm();
static void start_pwm(TIM_HandleTypeDef *htim);
static void sync_timers(TIM_HandleTypeDef *htim_a, TIM_HandleTypeDef *htim_b,
                        uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset);
// IRQ Callbacks (are all public)
// Measurement and calibrationa

// Test functions

static void FOC_voltage_loop(Motor &motor, float v_d, float v_q);
// Main motor control

// Motor thread (is public)

/* Function implementations --------------------------------------------------*/



//--------------------------------
// Initalisation
//--------------------------------

// Initalises the low level motor control and then starts the motor control
// threads
void initMotorControl() {
    // Init gate drivers
    DRV8301_setup(*Motor::getMotorByID(0));
    DRV8301_setup(*Motor::getMotorByID(1));

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
static void DRV8301_setup(Motor &motor) {
    DRV8301_Obj *gate_driver = &(motor.gate_driver);
    DRV_SPI_8301_Vars_t *local_regs = &(motor.gate_driver_regs);

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
            motor.phase_current_rev_gain = 1.0f / 10.0f;
            break;
        case DRV8301_ShuntAmpGain_20VpV:
            motor.phase_current_rev_gain = 1.0f / 20.0f;
            break;
        case DRV8301_ShuntAmpGain_40VpV:
            motor.phase_current_rev_gain = 1.0f / 40.0f;
            break;
        case DRV8301_ShuntAmpGain_80VpV:
            motor.phase_current_rev_gain = 1.0f / 80.0f;
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
// Utility
//--------------------------------



//--------------------------------
// IRQ Callbacks
//--------------------------------

// This is the callback from the ADC that we expect after the PWM has triggered
// an ADC conversion.
// TODO: Document how the phasing is done, link to timing diagram
void pwmTrigADC_cb(ADC_HandleTypeDef *hadc, bool injected) {
#define calib_tau 0.2f  //@TOTO make more easily configurable
    float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

    // Ensure ADCs are expected ones to simplify the logic below
    if (!(hadc == &hadc2 || hadc == &hadc3)) {
        Error::globalFault(ERROR_ADC_FAILED);
        return;
    };

    // Motor 0 is on Timer 1, which triggers ADC 2 and 3 on an injected
    // conversion
    // Motor 1 is on Timer 8, which triggers ADC 2 and 3 on a regular conversion
    // If the corresponding timer is counting up, we just sampled in SVM vector
    // 0, i.e. real current
    // If we are counting down, we just sampled in SVM vector 7, with zero
    // current
    Motor *motor = injected ? Motor::getMotorByID(0) : Motor::getMotorByID(1);
    bool counting_down = motor->motor_timer->Instance->CR1 & TIM_CR1_DIR;

    bool current_meas_not_DC_CAL;
    if (motor == Motor::getMotorByID(1) && counting_down) {
        // We are measuring M1 DC_CAL here
        current_meas_not_DC_CAL = false;
        // Load next timings for M0 (only once is sufficient)
        if (hadc == &hadc2) {
            Motor::getMotorByID(0)->motor_timer->Instance->CCR1 = Motor::getMotorByID(0)->next_timings[0];
            Motor::getMotorByID(0)->motor_timer->Instance->CCR2 = Motor::getMotorByID(0)->next_timings[1];
            Motor::getMotorByID(0)->motor_timer->Instance->CCR3 = Motor::getMotorByID(0)->next_timings[2];
        }
        // Check the timing of the sequencing
        motor->checkTiming();
    } else if (motor == Motor::getMotorByID(0) && !counting_down) {
        // We are measuring M0 current here
        current_meas_not_DC_CAL = true;
        // Load next timings for M1 (only once is sufficient)
        if (hadc == &hadc2) {
            Motor::getMotorByID(1)->motor_timer->Instance->CCR1 = Motor::getMotorByID(1)->next_timings[0];
            Motor::getMotorByID(1)->motor_timer->Instance->CCR2 = Motor::getMotorByID(1)->next_timings[1];
            Motor::getMotorByID(1)->motor_timer->Instance->CCR3 = Motor::getMotorByID(1)->next_timings[2];
        }
        // Check the timing of the sequencing
        motor->checkTiming();
    } else if (motor == Motor::getMotorByID(1) && !counting_down) {
        // We are measuring M1 current here
        current_meas_not_DC_CAL = true;
        // Check the timing of the sequencing
        motor->checkTiming();
    } else if (motor == Motor::getMotorByID(0) && counting_down) {
        // We are measuring M0 DC_CAL here
        current_meas_not_DC_CAL = false;
        // Check the timing of the sequencing
        motor->checkTiming();
    } else {
        Error::globalFault(ERROR_PWM_SRC_FAIL);
        return;
    }

    uint32_t ADCValue;
    if (injected) {
        ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    } else {
        ADCValue = HAL_ADC_GetValue(hadc);
    }
    float current = motor->phaseCurrentFromADCVal(ADCValue);

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
            osSignalSet(motor->motorThread, M_SIGNAL_PH_CURRENT_MEAS);
    } else {
        // DC_CAL measurement
        if (hadc == &hadc2) {
            motor->DC_calib.phB += (current - motor->DC_calib.phB) * calib_filter_k;
        } else {
            motor->DC_calib.phC += (current - motor->DC_calib.phC) * calib_filter_k;
        }
    }
}

//--------------------------------
// Test functions
//--------------------------------

// TODO integrate as mode in main control loop
static void FOC_voltage_loop(Motor &motor, float v_d, float v_q) {
    for (;;) {
        osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
        motor.updateRotor();

        float phase = motor.getRotorPhase();
        float c = arm_cos_f32(phase);
        float s = arm_sin_f32(phase);
        float v_alpha = c * v_d - s * v_q;
        float v_beta = c * v_q + s * v_d;
        motor.queueVoltageTimings(v_alpha, v_beta);

        // Check we meet deadlines after queueing
        motor.last_cpu_time = motor.checkTiming();
        if (!(motor.last_cpu_time < motor.control_deadline)) {
            motor.error = ERROR_FOC_VOLTAGE_TIMING;
            return;
        }
    }
}


//--------------------------------
// Motor thread
//--------------------------------

void motorThread() {
    Motor* motor = new Motor();  // Not a problem as long as we don't deallocate it
    motor->motorThread = osThreadGetId();
    motor->thread_ready = true;

    for (;;) {
        if (motor->do_calibration) {
            __HAL_TIM_MOE_ENABLE(motor->motor_timer);                   // enable pwm outputs
            motor->calibrateMotor();
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor->motor_timer);  // disables pwm outputs
            motor->do_calibration = false;
        }

        if (motor->calibration_ok && motor->enable_control) {
            motor->enable_step_dir = true;
            __HAL_TIM_MOE_ENABLE(motor->motor_timer);

            bool spin_up_ok = true;
            if (motor->rotor_mode == ROTOR_MODE_SENSORLESS)
                spin_up_ok = motor->spinUpSensorless();
            if (spin_up_ok)
                motor->controlMotorLoop();  // This doesn't return until motor->enable_control is false or there's an error.

            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor->motor_timer);
            motor->enable_step_dir = false;

            if (motor->enable_control) {  // if control is still enabled, we exited because of error
                motor->calibration_ok = false;
                motor->enable_control = false;
            }
        }

        motor->queueVoltageTimings(0.0f, 0.0f);
        osDelay(100);
    }
    motor->thread_ready = false;
}