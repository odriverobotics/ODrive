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
const float adc_full_scale = (float)(1 << 12);
const float adc_ref_voltage = 3.3f;
/* Global variables ----------------------------------------------------------*/

// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;
bool brake_resistor_armed = false;
/* Private constant data -----------------------------------------------------*/
static const GPIO_TypeDef* GPIOs_to_samp[] = { GPIOA, GPIOB, GPIOC };
static const int num_GPIO = sizeof(GPIOs_to_samp) / sizeof(GPIOs_to_samp[0]); 
/* Private variables ---------------------------------------------------------*/

// Two motors, sampling port A,B,C (coherent with current meas timing)
static uint16_t GPIO_port_samples [2][num_GPIO];
/* CPU critical section helpers ----------------------------------------------*/

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

// @brief Floats ALL phases immediately and disarms both motors and the brake resistor.
void low_level_fault(Motor::Error_t error) {
    // Disable all motors NOW!
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        safety_critical_disarm_motor_pwm(axes[i]->motor_);
        axes[i]->motor_.error_ |= error;
    }

    safety_critical_disarm_brake_resistor();
}

// @brief Kicks off the arming process of the motor.
// All calls to this function must clearly originate
// from user input.
void safety_critical_arm_motor_pwm(Motor& motor) {
    uint32_t mask = cpu_enter_critical();
    if (brake_resistor_armed) {
        motor.armed_state_ = Motor::ARMED_STATE_WAITING_FOR_TIMINGS;
    }
    cpu_exit_critical(mask);
}

// @brief Disarms the motor PWM.
// After calling this function, it is guaranteed that all three
// motor phases are floating and will not be enabled again until
// safety_critical_arm_motor_phases is called.
// @returns true if the motor was in a state other than disarmed before
bool safety_critical_disarm_motor_pwm(Motor& motor) {
    uint32_t mask = cpu_enter_critical();
    bool was_armed = motor.armed_state_ != Motor::ARMED_STATE_DISARMED;
    motor.armed_state_ = Motor::ARMED_STATE_DISARMED;
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor.hw_config_.timer);
    cpu_exit_critical(mask);
    return was_armed;
}

// @brief Updates the phase timings unless the motor is disarmed.
//
// If this is called at a rate higher than the motor's timer period,
// the actual PMW timings on the pins can be undefined for up to one
// timer period.
void safety_critical_apply_motor_pwm_timings(Motor& motor, uint16_t timings[3]) {
    uint32_t mask = cpu_enter_critical();
    if (!brake_resistor_armed) {
        motor.armed_state_ = Motor::ARMED_STATE_DISARMED;
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
    cpu_exit_critical(mask);
}

// @brief Arms the brake resistor
void safety_critical_arm_brake_resistor() {
    uint32_t mask = cpu_enter_critical();
    brake_resistor_armed = true;
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    cpu_exit_critical(mask);
}

// @brief Disarms the brake resistor and by extension
// all motor PWM outputs.
// After calling this, the brake resistor can only be armed again
// by calling safety_critical_arm_brake_resistor().
void safety_critical_disarm_brake_resistor() {
    uint32_t mask = cpu_enter_critical();
    brake_resistor_armed = false;
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        safety_critical_disarm_motor_pwm(axes[i]->motor_);
    }
    cpu_exit_critical(mask);
}

// @brief Updates the brake resistor PWM timings unless
// the brake resistor is disarmed.
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on) {
    if (high_on - low_off < TIM_APB1_DEADTIME_CLOCKS)
        low_level_fault(Motor::ERROR_BRAKE_DEADTIME_VIOLATION);
    uint32_t mask = cpu_enter_critical();
    if (brake_resistor_armed) {
        // Safe update of low and high side timings
        // To avoid race condition, first reset timings to safe state
        // ch3 is low side, ch4 is high side
        htim2.Instance->CCR3 = 0;
        htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
        htim2.Instance->CCR3 = low_off;
        htim2.Instance->CCR4 = high_on;
    }
    cpu_exit_critical(mask);
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
    sync_timers(&htim1, &htim8, TIM_CLOCKSOURCE_ITR0, TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128,
            &htim13);

    // Motor output starts in the disabled state
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    // Enable the update interrupt (used to coherently sample GPIO)
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

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
                 uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset,
                 TIM_HandleTypeDef* htim_refbase) {
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
    // Set and start reference timebase timer (if used)
    if (htim_refbase) {
        htim_refbase->Instance->CNT = count_offset;
        htim_refbase->Instance->CR1 |= (TIM_CR1_CEN); // start
    }
    // Start Timer a
    htim_a->Instance->CR1 |= (TIM_CR1_CEN);
    // Restore timer configs
    htim_a->Instance->CR2 = CR2_store;
    htim_b->Instance->SMCR = SMCR_store;
    // restore output
    htim_a->Instance->BDTR |= MOE_store_a;
    htim_b->Instance->BDTR |= MOE_store_b;
}

// @brief ADC1 measurements are written to this buffer by DMA
uint16_t adc_measurements_[ADC_CHANNEL_COUNT] = { 0 };

// @brief Starts the general purpose ADC on the ADC1 peripheral.
// The measured ADC voltages can be read with get_adc_voltage().
//
// ADC1 is set up to continuously sample all channels 0 to 15 in a
// round-robin fashion.
// DMA is used to copy the measured 12-bit values to adc_measurements_.
//
// The injected (high priority) channel of ADC1 is used to sample vbus_voltage.
// This conversion is triggered by TIM1 at the frequency of the motor control loop.
void start_general_purpose_adc() {
    ADC_ChannelConfTypeDef sConfig;

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        _Error_Handler((char*)__FILE__, __LINE__);
    }

    // Set up sampling sequence (channel 0 ... channel 15)
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    for (uint32_t channel = 0; channel < ADC_CHANNEL_COUNT; ++channel) {
        sConfig.Channel = channel << ADC_CR1_AWDCH_Pos;
        sConfig.Rank = channel + 1; // rank numbering starts at 1
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            _Error_Handler((char*)__FILE__, __LINE__);
    }

    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(adc_measurements_), ADC_CHANNEL_COUNT);
}

// @brief Returns the ADC voltage associated with the specified pin.
// GPIO_set_to_analog() must be called first to put the Pin into
// analog mode.
// Returns NaN if the pin has no associated ADC1 channel.
//
// On ODrive 3.3 and 3.4 the following pins can be used with this function:
//  GPIO_1, GPIO_2, GPIO_3, GPIO_4 and some pins that are connected to
//  on-board sensors (M0_TEMP, M1_TEMP, AUX_TEMP)
//
// The ADC values are sampled in background at ~30kHz without
// any CPU involvement.
//
// Details: each of the 16 conversion takes (15+26) ADC clock
// cycles and the ADC, so the update rate of the entire sequence is:
//  21000kHz / (15+26) / 16 = 32kHz
// The true frequency is slightly lower because of the injected vbus
// measurements
float get_adc_voltage(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin) {
    uint32_t channel = UINT32_MAX;
    if (GPIO_port == GPIOA) {
        if (GPIO_pin == GPIO_PIN_0)
            channel = 0;
        else if (GPIO_pin == GPIO_PIN_1)
            channel = 1;
        else if (GPIO_pin == GPIO_PIN_2)
            channel = 2;
        else if (GPIO_pin == GPIO_PIN_3)
            channel = 3;
        else if (GPIO_pin == GPIO_PIN_4)
            channel = 4;
        else if (GPIO_pin == GPIO_PIN_5)
            channel = 5;
        else if (GPIO_pin == GPIO_PIN_6)
            channel = 6;
        else if (GPIO_pin == GPIO_PIN_7)
            channel = 7;
    } else if (GPIO_port == GPIOB) {
        if (GPIO_pin == GPIO_PIN_0)
            channel = 8;
        else if (GPIO_pin == GPIO_PIN_1)
            channel = 9;
    } else if (GPIO_port == GPIOC) {
        if (GPIO_pin == GPIO_PIN_0)
            channel = 10;
        else if (GPIO_pin == GPIO_PIN_1)
            channel = 11;
        else if (GPIO_pin == GPIO_PIN_2)
            channel = 12;
        else if (GPIO_pin == GPIO_PIN_3)
            channel = 13;
        else if (GPIO_pin == GPIO_PIN_4)
            channel = 14;
        else if (GPIO_pin == GPIO_PIN_5)
            channel = 15;
    }
    if (channel < ADC_CHANNEL_COUNT)
        return ((float)adc_measurements_[channel]) * (adc_ref_voltage / adc_full_scale);
    else
        return 0.0f / 0.0f; // NaN
}

//--------------------------------
// IRQ Callbacks
//--------------------------------

void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected) {
    static const float voltage_scale = adc_ref_voltage * VBUS_S_DIVIDER_RATIO / adc_full_scale;
    // Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    vbus_voltage = ADCValue * voltage_scale;
    if (axes[0] && !axes[0]->error_ && axes[1] && !axes[1]->error_) {
        if (oscilloscope_pos >= OSCILLOSCOPE_SIZE)
            oscilloscope_pos = 0;
        oscilloscope[oscilloscope_pos++] = vbus_voltage;
    }
}

static void decode_hall_samples(Encoder& enc, uint16_t GPIO_samples[num_GPIO]) {
    GPIO_TypeDef* hall_ports[] = {
        enc.hw_config_.hallC_port,
        enc.hw_config_.hallB_port,
        enc.hw_config_.hallA_port,
    };
    uint16_t hall_pins[] = {
        enc.hw_config_.hallC_pin,
        enc.hw_config_.hallB_pin,
        enc.hw_config_.hallA_pin,
    };

    uint8_t hall_state = 0x0;
    for (int i = 0; i < 3; ++i) {
        int port_idx = 0;
        for (;;) {
            auto port = GPIOs_to_samp[port_idx];
            if (port == hall_ports[i])
                break;
            ++port_idx;
        }

        hall_state <<= 1;
        hall_state |= (GPIO_samples[port_idx] & hall_pins[i]) ? 1 : 0;
    }

    enc.hall_state_ = hall_state;
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
    int axis_num = injected ? 0 : 1;
    Axis& other_axis = injected ? *axes[1] : *axes[0];
    bool counting_down = axis.motor_.hw_config_.timer->Instance->CR1 & TIM_CR1_DIR;
    bool current_meas_not_DC_CAL = !counting_down;

    // Check the timing of the sequencing
    if (current_meas_not_DC_CAL)
        axis.motor_.log_timing(Motor::TIMING_LOG_ADC_CB_I);
    else
        axis.motor_.log_timing(Motor::TIMING_LOG_ADC_CB_DC);

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
        // Prepare hall readings
        // TODO move this to inside encoder update function
        decode_hall_samples(axis.encoder_, GPIO_port_samples[axis_num]);
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

void tim_update_cb(TIM_HandleTypeDef* htim) {
    
    // If the corresponding timer is counting up, we just sampled in SVM vector 0, i.e. real current
    // If we are counting down, we just sampled in SVM vector 7, with zero current
    bool counting_down = htim->Instance->CR1 & TIM_CR1_DIR;
    if (counting_down)
        return;
    
    int sample_ch;
    Axis* axis;
    if (htim == &htim1) {
        sample_ch = 0;
        axis = axes[0];
    } else if (htim == &htim8) {
        sample_ch = 1;
        axis = axes[1];
    } else {
        low_level_fault(Motor::ERROR_UNEXPECTED_TIMER_CALLBACK);
        return;
    }

    axis->encoder_.sample_now();

    for (int i = 0; i < num_GPIO; ++i) {
        GPIO_port_samples[sample_ch][i] = GPIOs_to_samp[i]->IDR;
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
        //shuts off all motors AND brake resistor, sets error code on all motors.
        low_level_fault(Motor::ERROR_BRAKE_CURRENT_OUT_OF_RANGE);
    }
}


/* RC PWM input --------------------------------------------------------------*/

// @brief Returns the ODrive GPIO number for a given
// TIM2 or TIM5 input capture channel number.
int tim_2_5_channel_num_to_gpio_num(int channel) {
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (channel >= 1 && channel <= 4) {
        // the channel numbers just happen to coincide with
        // the GPIO numbers
        return channel;
    } else {
        return -1;
    }
#else
    // Only ch4 is available on v3.2
    if (channel == 4) {
        return 4;
    } else {
        return -1;
    }
#endif
}
// @brief Returns the TIM2 or TIM5 channel number
// for a given GPIO number.
uint32_t gpio_num_to_tim_2_5_channel(int gpio_num) {
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    switch (gpio_num) {
        case 1: return TIM_CHANNEL_1;
        case 2: return TIM_CHANNEL_2;
        case 3: return TIM_CHANNEL_3;
        case 4: return TIM_CHANNEL_4;
        default: return 0;
    }
#else
    // Only ch4 is available on v3.2
    if (gpio_num == 4) {
        return TIM_CHANNEL_4;
    } else {
        return 0;
    }
#endif
}

void pwm_in_init() {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;

    TIM_IC_InitTypeDef sConfigIC;
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    for (int gpio_num = 1; gpio_num <= 4; ++gpio_num) {
#else
    int gpio_num = 4; {
#endif
        if (is_endpoint_ref_valid(board_config.pwm_mappings[gpio_num - 1].endpoint)) {
            GPIO_InitStruct.Pin = get_gpio_pin_by_pin(gpio_num);
            HAL_GPIO_DeInit(get_gpio_port_by_pin(gpio_num), get_gpio_pin_by_pin(gpio_num));
            HAL_GPIO_Init(get_gpio_port_by_pin(gpio_num), &GPIO_InitStruct);
            HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, gpio_num_to_tim_2_5_channel(gpio_num));
            HAL_TIM_IC_Start_IT(&htim5, gpio_num_to_tim_2_5_channel(gpio_num));
        }
    }
}

//TODO: These expressions have integer division by 1MHz, so it will be incorrect for clock speeds of not-integer MHz
#define TIM_2_5_CLOCK_HZ        TIM_APB1_CLOCK_HZ
#define PWM_MIN_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 1000UL) // 1ms high is considered full reverse
#define PWM_MAX_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2000UL) // 2ms high is considered full forward
#define PWM_MIN_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 500UL) // ignore high periods shorter than 0.5ms
#define PWM_MAX_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2500UL) // ignore high periods longer than 2.5ms
#define PWM_INVERT_INPUT        false

void handle_pulse(int gpio_num, uint32_t high_time) {
    if (high_time < PWM_MIN_LEGAL_HIGH_TIME || high_time > PWM_MAX_LEGAL_HIGH_TIME)
        return;

    if (high_time < PWM_MIN_HIGH_TIME)
        high_time = PWM_MIN_HIGH_TIME;
    if (high_time > PWM_MAX_HIGH_TIME)
        high_time = PWM_MAX_HIGH_TIME;
    float fraction = (float)(high_time - PWM_MIN_HIGH_TIME) / (float)(PWM_MAX_HIGH_TIME - PWM_MIN_HIGH_TIME);
    float value = board_config.pwm_mappings[gpio_num - 1].min +
                  (fraction * (board_config.pwm_mappings[gpio_num - 1].max - board_config.pwm_mappings[gpio_num - 1].min));

    Endpoint* endpoint = get_endpoint(board_config.pwm_mappings[gpio_num - 1].endpoint);
    if (!endpoint)
        return;

    endpoint->set_from_float(value);
}

void pwm_in_cb(int channel, uint32_t timestamp) {
    static uint32_t last_timestamp[GPIO_COUNT] = { 0 };
    static bool last_pin_state[GPIO_COUNT] = { false };
    static bool last_sample_valid[GPIO_COUNT] = { false };

    int gpio_num = tim_2_5_channel_num_to_gpio_num(channel);
    if (gpio_num < 1 || gpio_num > GPIO_COUNT)
        return;
    bool current_pin_state = HAL_GPIO_ReadPin(get_gpio_port_by_pin(gpio_num), get_gpio_pin_by_pin(gpio_num)) != GPIO_PIN_RESET;

    if (last_sample_valid[gpio_num - 1]
        && (last_pin_state[gpio_num - 1] != PWM_INVERT_INPUT)
        && (current_pin_state == PWM_INVERT_INPUT)) {
        handle_pulse(gpio_num, timestamp - last_timestamp[gpio_num - 1]);
    }

    last_timestamp[gpio_num - 1] = timestamp;
    last_pin_state[gpio_num - 1] = current_pin_state;
    last_sample_valid[gpio_num - 1] = true;
}


/* Analog speed control input */

static void update_analog_endpoint(const struct PWMMapping_t *map, int gpio)
{
    float fraction = get_adc_voltage(get_gpio_port_by_pin(gpio), get_gpio_pin_by_pin(gpio)) / 3.3f;
    float value = map->min + (fraction * (map->max - map->min));
    get_endpoint(map->endpoint)->set_from_float(value);
}

static void analog_polling_thread(void *)
{
    while (true) {
        for (int i = 0; i < GPIO_COUNT; i++) {
            struct PWMMapping_t *map = &board_config.analog_mappings[i];

            if (is_endpoint_ref_valid(map->endpoint))
                update_analog_endpoint(map, i + 1);
        }
        osDelay(10);
    }
}

void start_analog_thread()
{
    osThreadDef(thread_def, analog_polling_thread, osPriorityLow, 0, 4*512);
    osThreadCreate(osThread(thread_def), NULL);
}
