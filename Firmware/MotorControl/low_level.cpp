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

//#include <adc.h>
//#include <gpio.hpp>
//#include <main.h>
//#include <spi.h>
//#include <tim.h>
#include <utils.h>
#include <stm32_tim.hpp>

#include "odrive_main.h"

/* Private defines -----------------------------------------------------------*/

// #define DEBUG_PRINT

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
bool brake_resistor_armed = false;
/* Private constant data -----------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

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
    for (size_t i = 0; i < n_axes; ++i) {
        safety_critical_disarm_motor_pwm(axes[i].motor_);
        axes[i].motor_.error_ |= error;
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
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&motor.timer_->htim);
    cpu_exit_critical(mask);
    return was_armed;
}

// @brief Updates the phase timings unless the motor is disarmed.
//
// If this is called at a rate higher than the motor's timer period,
// the actual PMW timings on the pins can be undefined for up to one
// timer period.
// TODO: update this and the description
void safety_critical_apply_motor_pwm_timings(Motor& motor, uint16_t period, uint16_t timings[3]) {
    uint32_t mask = cpu_enter_critical();
    if (!brake_resistor_armed) {
        motor.armed_state_ = Motor::ARMED_STATE_DISARMED;
    }

    motor.is_updating_pwm_timings_ = true;
    motor.timer_->htim.Instance->CCR1 = timings[0];
    motor.timer_->htim.Instance->CCR2 = timings[1];
    motor.timer_->htim.Instance->CCR3 = timings[2];
    motor.timer_->htim.Instance->ARR = period;
    motor.pwm_control_deadline_ = period - 1;
    motor.is_updating_pwm_timings_ = false;

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
    tim2.htim.Instance->CCR3 = 0;
    tim2.htim.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    cpu_exit_critical(mask);
}

// @brief Disarms the brake resistor and by extension
// all motor PWM outputs.
// After calling this, the brake resistor can only be armed again
// by calling safety_critical_arm_brake_resistor().
void safety_critical_disarm_brake_resistor() {
    uint32_t mask = cpu_enter_critical();
    brake_resistor_armed = false;
    tim2.htim.Instance->CCR3 = 0;
    tim2.htim.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    for (size_t i = 0; i < n_axes; ++i) {
        safety_critical_disarm_motor_pwm(axes[i].motor_);
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
        tim2.htim.Instance->CCR3 = 0;
        tim2.htim.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
        tim2.htim.Instance->CCR3 = low_off;
        tim2.htim.Instance->CCR4 = high_on;
    }
    cpu_exit_critical(mask);
}

/* Function implementations --------------------------------------------------*/

//--------------------------------
// IRQ Callbacks
//--------------------------------


// @brief Sums up the Ibus contribution of each motor and updates the
// brake resistor PWM accordingly.
void update_brake_current() {
    float Ibus_sum = 0.0f;
    for (size_t i = 0; i < n_axes; ++i) {
        if (axes[i].motor_.armed_state_ == Motor::ARMED_STATE_ARMED) {
            Ibus_sum += axes[i].motor_.current_control_.Ibus;
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

/* Analog speed control input */

static void update_analog_endpoint(const struct PWMMapping_t *map, uint32_t gpio_num)
{
    float fraction = get_adc_voltage(gpio_num) / 3.3f;
    float value = map->min + (fraction * (map->max - map->min));
    get_endpoint(map->endpoint)->set_from_float(value);
}

static void analog_polling_thread(void *)
{
    while (true) {
        for (uint32_t i = 0; i < num_gpios; i++) {
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
