/* Includes ------------------------------------------------------------------*/

// Because of broken cmsis_os.h, we need to include arm_math first,
// otherwise chip specific defines are ommited
#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4
#include <arm_math.h>

#include <cmsis_os.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <adc.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <utils.h>

#include "odrive_main.h"

/* Private defines -----------------------------------------------------------*/

// #define DEBUG_PRINT

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;
bool brake_resistor_armed_ = false;

/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* CPU critical section helpers ----------------------------------------------*/

static inline uint8_t cpu_enter_critical() {
    uint8_t status_register;
    asm (
        "MRS   R0, PRIMASK\n\t"
        "CPSID I\n\t"
        "STRB R0, %[output]"
        : [output] "=m" (status_register) :: "r0"
    );
    return status_register;
}
 
static inline void cpu_exit_critical(uint8_t status_register) {
    asm (
        "ldrb r0, %[input]\n\t"
        "msr PRIMASK,r0;\n\t"
        ::[input] "m" (status_register) : "r0"
    );
}

/* Safety critical functions -------------------------------------------------*/

/*
* This section contains all accesses to safety critical hardware registers.
* Specifically, these registers:
*   Motor0 PWMs:
*     Timer1.MOE (master output enabled)
*     Timer1.CCR1 (counter compare register 1)
*     Timer1.CCR2 (counter compare register 2)
*     Timer1.CCR3 (counter compare register 3)
*   Motor1 PWMs:
*     Timer8.MOE (master output enabled)
*     Timer8.CCR1 (counter compare register 1)
*     Timer8.CCR2 (counter compare register 2)
*     Timer8.CCR3 (counter compare register 3)
*   Brake resistor PWM:
*     Timer2.CCR3 (counter compare register 3)
*     Timer2.CCR4 (counter compare register 4)
* 
* The following assumptions are made:
*   - The hardware operates as described in the datasheet:
*     http://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
*     This assumption also requires for instance that there are no radiation
*     caused hardware errors.
*   - After startup, all variables used in this section are exclusively modified
*     by the code in this section (this excludes function parameters)
*     This assumption also requires that there is no memory corruption.
*   - This code is compiled by a C standard compliant compiler.
*
* Furthermore:
*   - Between calls to safety_critical_arm_motor_pwm and
*     safety_critical_disarm_motor_pwm the motor's Ibus current is
*     set to the correct value and update_brake_resistor is called
*     at a high rate.
*/


// @brief Kicks off the arming process of the motor.
// All calls to this function must clearly originate
// from user input.
void safety_critical_arm_motor_pwm(Motor& motor) {
    uint8_t sr = cpu_enter_critical();
    if (brake_resistor_armed_) {
        motor.armed_state_ = Motor::ARMED_STATE_WAITING_FOR_TIMINGS;
    }
    cpu_exit_critical(sr);
}

// @brief Disarms the motor PWM.
// After calling this function, it is guaranteed that all three
// motor phases are floating and will not be enabled again until
// safety_critical_arm_motor_phases is called.
// @returns true if the motor was in a state other than disarmed before
bool safety_critical_disarm_motor_pwm(Motor& motor) {
    uint8_t sr = cpu_enter_critical();
    bool was_armed = motor.armed_state_ != Motor::ARMED_STATE_DISARMED;
    motor.armed_state_ = Motor::ARMED_STATE_DISARMED;
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor.hw_config_.timer);
    cpu_exit_critical(sr);
    return was_armed;
}

// @brief Updates the phase timings unless the motor is disarmed.
//
// If this is called at a rate higher than the motor's timer period,
// the actual PMW timings on the pins can be undefined for up to one
// timer period.
void safety_critical_apply_motor_pwm_timings(Motor& motor, uint16_t timings[3]) {
    uint8_t sr = cpu_enter_critical();
    if (!brake_resistor_armed_) {
        motor.armed_state_ = Motor::ARMED_STATE_ARMED;
    }

    motor.hw_config_.timer->Instance->CCR1 = timings[0];
    motor.hw_config_.timer->Instance->CCR2 = timings[1];
    motor.hw_config_.timer->Instance->CCR3 = timings[2];

    if (motor.armed_state_ == Motor::ARMED_STATE_WAITING_FOR_TIMINGS) {
        // timings were just loaded into the timer registers
        // the timer register are buffered, so they won't have an effect
        // on the output just yet so we need to wait until the next
        // interrupt before we actually enable the output
        motor.armed_state_ = Motor::ARMED_STATE_WAITING_FOR_UPDATE;
    } else if (motor.armed_state_ == Motor::ARMED_STATE_WAITING_FOR_UPDATE) {
        // now we waited long enough. Enter armed state and
        // enable the actual PWM outputs.
        motor.armed_state_ = Motor::ARMED_STATE_ARMED;
        __HAL_TIM_MOE_ENABLE(motor.hw_config_.timer);  // enable pwm outputs
    } else if (motor.armed_state_ == Motor::ARMED_STATE_ARMED) {
        // nothing to do, PWM is running, all good
    } else {
        // unknown state oh no
        safety_critical_disarm_motor_pwm(motor);
    }
    cpu_exit_critical(sr);
}

// @brief Arms the brake resistor
void safety_critical_arm_brake_resistor() {
    uint8_t sr = cpu_enter_critical();
    brake_resistor_armed_ = true;
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    cpu_exit_critical(sr);
}

// @brief Disarms the brake resistor and by extension
// all motor PWM outputs.
// After calling this, the brake resistor can only be armed again
// by calling safety_critical_arm_brake_resistor().
void safety_critical_disarm_brake_resistor() {
    uint8_t sr = cpu_enter_critical();
    brake_resistor_armed_ = false;
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        safety_critical_disarm_motor_pwm(axes[i]->motor_);
    }
    cpu_exit_critical(sr);
}

// @brief Updates the brake resistor PWM timings unless
// the brake resistor is disarmed.
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on) {
    if (high_on - low_off > TIM_APB1_DEADTIME_CLOCKS)
        for(;;);
    uint8_t sr = cpu_enter_critical();
    if (brake_resistor_armed_) {
        // Safe update of low and high side timings
        // To avoid race condition, first reset timings to safe state
        // ch3 is low side, ch4 is high side
        htim2.Instance->CCR3 = 0;
        htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
        htim2.Instance->CCR3 = low_off;
        htim2.Instance->CCR4 = high_on;
    }
    cpu_exit_critical(sr);
}

/* Function implementations --------------------------------------------------*/

void start_adc_pwm() {
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
    sync_timers(&htim1, &htim8, TIM_CLOCKSOURCE_ITR0, TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128);

    // Motor output starts in the disabled state
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    // Start brake resistor PWM in floating output configuration
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    // Disarm motors and arm brake resistor
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        safety_critical_disarm_motor_pwm(axes[i]->motor_);
    }
    safety_critical_arm_brake_resistor();
}

void start_pwm(TIM_HandleTypeDef* htim) {
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

void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
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

// @brief Floats ALL phases immediately and disarms both motors and the brake resistor.
void low_level_fault(Motor::Error_t error) {
    // Disable all motors NOW!
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        safety_critical_disarm_motor_pwm(axes[i]->motor_);
        axes[i]->motor_.error_ |= error;
    }
    
    safety_critical_disarm_brake_resistor();
}

//--------------------------------
// IRQ Callbacks
//--------------------------------


void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected) {
    static const float voltage_scale = 3.3f * VBUS_S_DIVIDER_RATIO / (float)(1 << 12);
    // Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    vbus_voltage = ADCValue * voltage_scale;
    if (axes[0] && !axes[0]->error_ && axes[1] && !axes[1]->error_) {
        if (oscilloscope_pos >= OSCILLOSCOPE_SIZE)
            oscilloscope_pos = 0;
        oscilloscope[oscilloscope_pos++] = vbus_voltage;
    }
}

// This is the callback from the ADC that we expect after the PWM has triggered an ADC conversion.
// TODO: Document how the phasing is done, link to timing diagram
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected) {
#define calib_tau 0.2f  //@TOTO make more easily configurable
    static const float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

    // Ensure ADCs are expected ones to simplify the logic below
    if (!(hadc == &hadc2 || hadc == &hadc3)) {
        low_level_fault(Motor::ERROR_ADC_FAILED);
        return;
    };

    // Motor 0 is on Timer 1, which triggers ADC 2 and 3 on an injected conversion
    // Motor 1 is on Timer 8, which triggers ADC 2 and 3 on a regular conversion
    // If the corresponding timer is counting up, we just sampled in SVM vector 0, i.e. real current
    // If we are counting down, we just sampled in SVM vector 7, with zero current
    Axis& axis = injected ? *axes[0] : *axes[1];
    Axis& other_axis = injected ? *axes[1] : *axes[0];
    bool counting_down = axis.motor_.hw_config_.timer->Instance->CR1 & TIM_CR1_DIR;

    bool current_meas_not_DC_CAL = !counting_down;
    bool update_timings = false;
    if (hadc == &hadc2) {
        if (&axis == axes[1] && counting_down)
            update_timings = true; // update timings of M0
        else if (&axis == axes[0] && !counting_down)
            update_timings = true; // update timings of M1
    }

    // Load next timings for the motor that we're not currently sampling
    if (update_timings) {
        if (!other_axis.motor_.next_timings_valid_) {
            // the motor control loop failed to update the timings in time
            // we must assume that it died and therefore float all phases
            bool was_armed = safety_critical_disarm_motor_pwm(other_axis.motor_);
            if (was_armed) {
                other_axis.motor_.error_ |= Motor::ERROR_CONTROL_DEADLINE_MISSED;
            }
        } else {
            other_axis.motor_.next_timings_valid_ = false;
            safety_critical_apply_motor_pwm_timings(
                other_axis.motor_, other_axis.motor_.next_timings_
            );
        }
        update_brake_current();
    }

    // Check the timing of the sequencing
    if (current_meas_not_DC_CAL)
        axis.motor_.log_timing(Motor::TIMING_LOG_ADC_CB_I);
    else
        axis.motor_.log_timing(Motor::TIMING_LOG_ADC_CB_DC);

    uint32_t ADCValue;
    if (injected) {
        ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    } else {
        ADCValue = HAL_ADC_GetValue(hadc);
    }
    float current = axis.motor_.phase_current_from_adcval(ADCValue);

    if (current_meas_not_DC_CAL) {
        // ADC2 and ADC3 record the phB and phC currents concurrently,
        // and their interrupts should arrive on the same clock cycle.
        // We dispatch the callbacks in order, so ADC2 will always be processed before ADC3.
        // Therefore we store the value from ADC2 and signal the thread that the
        // measurement is ready when we receive the ADC3 measurement

        // return or continue
        if (hadc == &hadc2) {
            axis.motor_.current_meas_.phB = current - axis.motor_.DC_calib_.phB;
            return;
        } else {
            axis.motor_.current_meas_.phC = current - axis.motor_.DC_calib_.phC;
        }
        // Trigger axis thread
        axis.signal_current_meas();
    } else {
        // DC_CAL measurement
        if (hadc == &hadc2) {
            axis.motor_.DC_calib_.phB += (current - axis.motor_.DC_calib_.phB) * calib_filter_k;
        } else {
            axis.motor_.DC_calib_.phC += (current - axis.motor_.DC_calib_.phC) * calib_filter_k;
        }
    }
}

// @brief Sums up the Ibus contribution of each motor and updates the
// brake resistor PWM accordingly.
void update_brake_current() {
    float Ibus_sum = 0.0f;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (axes[i]->motor_.armed_state_ == Motor::ARMED_STATE_ARMED) {
            Ibus_sum += axes[i]->motor_.current_control_.Ibus;
        }
    }
    float brake_current = -Ibus_sum;
    // Clip negative values to 0.0f
    if (brake_current < 0.0f) brake_current = 0.0f;
    float brake_duty = brake_current * board_config.brake_resistance / vbus_voltage;

    // Duty limit at 90% to allow bootstrap caps to charge
    // If brake_duty is NaN, this expression will also evaluate to false
    if ((brake_duty >= 0.0f) && (brake_duty <= 0.9f)) {
        int high_on = static_cast<int>(TIM_APB1_PERIOD_CLOCKS * (1.0f - brake_duty));
        int low_off = high_on - TIM_APB1_DEADTIME_CLOCKS;
        if (low_off < 0) low_off = 0;
        safety_critical_apply_brake_resistor_timings(low_off, high_on);
    } else {
        safety_critical_disarm_brake_resistor();
    }
}
