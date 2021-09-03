/* Includes ------------------------------------------------------------------*/

#include <board.h>

#include <cmsis_os.h>
#include <cmath>
#include <stdint.h>
#include <stdlib.h>

#include <adc.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <utils.hpp>

#include "odrive_main.h"

/* Private defines -----------------------------------------------------------*/

// #define DEBUG_PRINT

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
constexpr float adc_full_scale = static_cast<float>(1UL << 12UL);
constexpr float adc_ref_voltage = 3.3f;
const uint32_t stack_size_analog_thread = 1024;  // Bytes
/* Global variables ----------------------------------------------------------*/

// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;
float ibus_ = 0.0f; // exposed for monitoring only
bool brake_resistor_armed = false;
bool brake_resistor_saturated = false;
float brake_resistor_current = 0.0f;
osThreadId analog_thread = 0;
/* Private constant data -----------------------------------------------------*/
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


// @brief Arms the brake resistor
void safety_critical_arm_brake_resistor() {
    CRITICAL_SECTION() {
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            axes[i].motor_.I_bus_ = 0.0f;
        }
        brake_resistor_armed = true;
#if HW_VERSION_MAJOR == 3
        htim2.Instance->CCR3 = 0;
        htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
#endif
    }
}

// @brief Disarms the brake resistor and by extension
// all motor PWM outputs.
// After calling this, the brake resistor can only be armed again
// by calling safety_critical_arm_brake_resistor().
void safety_critical_disarm_brake_resistor() {
    bool brake_resistor_was_armed = brake_resistor_armed;

    CRITICAL_SECTION() {
        brake_resistor_armed = false;
#if HW_VERSION_MAJOR == 3
        htim2.Instance->CCR3 = 0;
        htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
#endif
    }

    // Check necessary to prevent infinite recursion
    if (brake_resistor_was_armed) {
        for (auto& axis: axes) {
            axis.motor_.disarm();
        }
    }
}

// @brief Updates the brake resistor PWM timings unless
// the brake resistor is disarmed.
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on) {
    if (high_on - low_off < TIM_APB1_DEADTIME_CLOCKS) {
        odrv.disarm_with_error(ODrive::ERROR_BRAKE_DEADTIME_VIOLATION);
    }

    CRITICAL_SECTION() {
        if (brake_resistor_armed) {
#if HW_VERSION_MAJOR == 3
            // Safe update of low and high side timings
            // To avoid race condition, first reset timings to safe state
            // ch3 is low side, ch4 is high side
            htim2.Instance->CCR3 = 0;
            htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
            htim2.Instance->CCR3 = low_off;
            htim2.Instance->CCR4 = high_on;
#endif
        }
    }
}

/* Function implementations --------------------------------------------------*/

void start_adc_pwm() {
    // Disarm motors
    for (auto& axis: axes) {
        axis.motor_.disarm();
    }

    for (Motor& motor: motors) {
        // Init PWM
        int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
        motor.timer_->Instance->CCR1 = half_load;
        motor.timer_->Instance->CCR2 = half_load;
        motor.timer_->Instance->CCR3 = half_load;

        // Enable PWM outputs (they are still masked by MOE though)
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_1);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_1);
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_2);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_2);
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_3);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_3);
    }

    // Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    // Warp field stabilize.
    osDelay(2);


    start_timers();


    // Start brake resistor PWM in floating output configuration
#if HW_VERSION_MAJOR == 3
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
#endif

    if (odrv.config_.enable_brake_resistor) {
        safety_critical_arm_brake_resistor();
    }
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
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        odrv.misconfigured_ = true; // TODO: this is a bit of an abuse of this flag
        return;
    }

    // Set up sampling sequence (channel 0 ... channel 15)
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    for (uint32_t channel = 0; channel < ADC_CHANNEL_COUNT; ++channel) {
        sConfig.Channel = channel << ADC_CR1_AWDCH_Pos;
        sConfig.Rank = channel + 1; // rank numbering starts at 1
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            odrv.misconfigured_ = true; // TODO: this is a bit of an abuse of this flag
            return;
        }
    }

    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(adc_measurements_), ADC_CHANNEL_COUNT);
}

// @brief Returns the ADC voltage associated with the specified pin.
// This only works if the GPIO was not used for anything else since bootup, otherwise
// it must be put to analog mode first.
// Returns -1.0f if the pin has no associated ADC1 channel.
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
float get_adc_voltage(Stm32Gpio gpio) {
    return get_adc_relative_voltage(gpio) * adc_ref_voltage;
}

float get_adc_relative_voltage(Stm32Gpio gpio) {
    const uint16_t channel = channel_from_gpio(gpio);
    return get_adc_relative_voltage_ch(channel);
}

// @brief Given a GPIO_port and pin return the associated adc_channel.
// returns UINT16_MAX if there is no adc_channel;
uint16_t channel_from_gpio(Stm32Gpio gpio) {
    uint32_t channel = UINT32_MAX;
    if (gpio.port_ == GPIOA) {
        if (gpio.pin_mask_ == GPIO_PIN_0)
            channel = 0;
        else if (gpio.pin_mask_ == GPIO_PIN_1)
            channel = 1;
        else if (gpio.pin_mask_ == GPIO_PIN_2)
            channel = 2;
        else if (gpio.pin_mask_ == GPIO_PIN_3)
            channel = 3;
        else if (gpio.pin_mask_ == GPIO_PIN_4)
            channel = 4;
        else if (gpio.pin_mask_ == GPIO_PIN_5)
            channel = 5;
        else if (gpio.pin_mask_ == GPIO_PIN_6)
            channel = 6;
        else if (gpio.pin_mask_ == GPIO_PIN_7)
            channel = 7;
    } else if (gpio.port_ == GPIOB) {
        if (gpio.pin_mask_ == GPIO_PIN_0)
            channel = 8;
        else if (gpio.pin_mask_ == GPIO_PIN_1)
            channel = 9;
    } else if (gpio.port_ == GPIOC) {
        if (gpio.pin_mask_ == GPIO_PIN_0)
            channel = 10;
        else if (gpio.pin_mask_ == GPIO_PIN_1)
            channel = 11;
        else if (gpio.pin_mask_ == GPIO_PIN_2)
            channel = 12;
        else if (gpio.pin_mask_ == GPIO_PIN_3)
            channel = 13;
        else if (gpio.pin_mask_ == GPIO_PIN_4)
            channel = 14;
        else if (gpio.pin_mask_ == GPIO_PIN_5)
            channel = 15;
    }
    return channel;
}

// @brief Given an adc channel return the voltage as a ratio of adc_ref_voltage
// returns -1.0f if the channel is not valid.
float get_adc_relative_voltage_ch(uint16_t channel) {
    if (channel < ADC_CHANNEL_COUNT)
        return (float)adc_measurements_[channel] / adc_full_scale;
    else
        return -1.0f;
}

//--------------------------------
// IRQ Callbacks
//--------------------------------

void vbus_sense_adc_cb(uint32_t adc_value) {
    constexpr float voltage_scale = adc_ref_voltage * VBUS_S_DIVIDER_RATIO / adc_full_scale;
    vbus_voltage = adc_value * voltage_scale;
}

// @brief Sums up the Ibus contribution of each motor and updates the
// brake resistor PWM accordingly.
void update_brake_current() {
    float Ibus_sum = 0.0f;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (axes[i].motor_.is_armed_) {
            Ibus_sum += axes[i].motor_.I_bus_;
        }
    }

    float brake_duty = 0.0f;
    float brake_current = 0.0f;
    if (odrv.config_.enable_brake_resistor) {
        if (!(odrv.config_.brake_resistance > 0.0f)) {
            odrv.disarm_with_error(ODrive::ERROR_INVALID_BRAKE_RESISTANCE);
            return;
        }
    
        // Don't start braking until -Ibus > regen_current_allowed
        brake_current = -Ibus_sum - odrv.config_.max_regen_current;
        brake_duty = brake_current * odrv.config_.brake_resistance / vbus_voltage;
        
        if (odrv.config_.enable_dc_bus_overvoltage_ramp && (odrv.config_.brake_resistance > 0.0f) && (odrv.config_.dc_bus_overvoltage_ramp_start < odrv.config_.dc_bus_overvoltage_ramp_end)) {
            brake_duty += std::max((vbus_voltage - odrv.config_.dc_bus_overvoltage_ramp_start) / (odrv.config_.dc_bus_overvoltage_ramp_end - odrv.config_.dc_bus_overvoltage_ramp_start), 0.0f);
        }

        if (is_nan(brake_duty)) {
            // Shuts off all motors AND brake resistor, sets error code on all motors.
            odrv.disarm_with_error(ODrive::ERROR_BRAKE_DUTY_CYCLE_NAN);
            return;
        }

        if (brake_duty >= 0.95f) {
            brake_resistor_saturated = true;
        }

        // Duty limit at 95% to allow bootstrap caps to charge
        brake_duty = std::clamp(brake_duty, 0.0f, 0.95f);

        // This cannot result in NaN (safe for race conditions) because we check
        // brake_resistance != 0 further up.
        brake_current = brake_duty * vbus_voltage / odrv.config_.brake_resistance;
        Ibus_sum += brake_duty * vbus_voltage / odrv.config_.brake_resistance;
    } else {
        brake_duty = 0;
    }

    brake_resistor_current = brake_current;
    ibus_ += odrv.ibus_report_filter_k_ * (Ibus_sum - ibus_);

    if (Ibus_sum > odrv.config_.dc_max_positive_current) {
        odrv.disarm_with_error(ODrive::ERROR_DC_BUS_OVER_CURRENT);
        return;
    }
    if (Ibus_sum < odrv.config_.dc_max_negative_current) {
        odrv.disarm_with_error(ODrive::ERROR_DC_BUS_OVER_REGEN_CURRENT);
        return;
    }
    
    int high_on = (int)(TIM_APB1_PERIOD_CLOCKS * (1.0f - brake_duty));
    int low_off = high_on - TIM_APB1_DEADTIME_CLOCKS;
    if (low_off < 0) low_off = 0;
    safety_critical_apply_brake_resistor_timings(low_off, high_on);
}


/* Analog speed control input */

static void update_analog_endpoint(const struct PWMMapping_t *map, int gpio)
{
    float fraction = get_adc_voltage(get_gpio(gpio)) / 3.3f;
    float value = map->min + (fraction * (map->max - map->min));
    fibre::set_endpoint_from_float(map->endpoint, value);
}

static void analog_polling_thread(void *)
{
    while (true) {
        for (int i = 0; i < GPIO_COUNT; i++) {
            struct PWMMapping_t *map = &odrv.config_.analog_mappings[i];

            if (fibre::is_endpoint_ref_valid(map->endpoint))
                update_analog_endpoint(map, i);
        }
        osDelay(10);
    }
}

void start_analog_thread() {
    osThreadDef(analog_thread_def, analog_polling_thread, osPriorityLow, 0, stack_size_analog_thread / sizeof(StackType_t));
    analog_thread = osThreadCreate(osThread(analog_thread_def), NULL);
}
