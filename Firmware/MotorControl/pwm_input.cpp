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

#define TIM_2_5_CLOCK_HZ        TIM_APB1_CLOCK_HZ
#define PWM_HIGH_STATE          true // Change to false to invert input

#define RC_MIN_HIGH_TIME        ((TIM_2_5_CLOCK_HZ / 1000000UL) * 1000UL) // 1ms high is considered full reverse
#define RC_MAX_HIGH_TIME        ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2000UL) // 2ms high is considered full forward
#define RC_MIN_LEGAL_HIGH_TIME  ((TIM_2_5_CLOCK_HZ / 1000000UL) * 500UL) // ignore high periods shorter than 0.5ms
#define RC_MAX_LEGAL_HIGH_TIME  ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2500UL) // ignore high periods longer than 2.5ms

#define PWM_INACTIVE_TIME        (((TIM_2_5_CLOCK_HZ / 1000UL) * 45UL)) // 45 milliseconds.  RC PWM should cycle every 20ms.

#define PWM_DEADBAND_MIN    0.495f // Values in the range [DEADBAND_MIN, DEADBAND_MAX] will output (min+max)/2
#define PWM_DEADBAND_MAX    0.505f // Values in the range [DEADBAND_MIN, DEADBAND_MAX] will output (min+max)/2

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static uint32_t last_active_timestamp[PWM_IN_COUNT] = { 0 }; // Timer ticks
static uint32_t last_LtoH_timestamp[PWM_IN_COUNT] = { 0 }; // Timer ticks
static uint32_t last_HtoL_timestamp[PWM_IN_COUNT] = { 0 }; // Timer ticks
static uint32_t last_period_110[PWM_IN_COUNT] = { 0 }; // Timer ticks
static uint32_t last_period_090[PWM_IN_COUNT] = { 0 }; // Timer ticks
static uint32_t last_period[PWM_IN_COUNT] = { 0 }; // Timer ticks
static bool last_pin_state[PWM_IN_COUNT] = { false };
static bool channel_active[PWM_IN_COUNT] = { false };
static float channel_value[PWM_IN_COUNT]; // Channel output value before modification by direction pin

// Debugging variables
static uint32_t valid_counts[PWM_IN_COUNT];
static uint32_t all_counts[PWM_IN_COUNT];

/* Function prototypes -------------------------------------------------------*/

void pwm_no_pulse_check_cb(void *argument);

/* Function implementations --------------------------------------------------*/

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
    for (int gpio_num = 1; gpio_num <= PWM_IN_COUNT; ++gpio_num) {
#else
    int gpio_num = 4; {
#endif  
        // Activate timer interrupt if the pin is being used as a PWM input
        bool pwm_dir_pin = false;
        for (uint8_t i = 0; i < PWM_IN_COUNT; ++i) {
            if (board_config.pwm_mappings[i].gpio_direction_pin == gpio_num) {
                pwm_dir_pin = true;
            }
        }
        if (is_endpoint_ref_valid(board_config.pwm_mappings[gpio_num - 1].endpoint) || pwm_dir_pin) {
            GPIO_InitStruct.Pin = get_gpio_pin_by_pin(gpio_num);
            HAL_GPIO_DeInit(get_gpio_port_by_pin(gpio_num), get_gpio_pin_by_pin(gpio_num));
            HAL_GPIO_Init(get_gpio_port_by_pin(gpio_num), &GPIO_InitStruct);
            HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, gpio_num_to_tim_2_5_channel(gpio_num));
            HAL_TIM_IC_Start_IT(&htim5, gpio_num_to_tim_2_5_channel(gpio_num));
        }
    }

    // Create and start a periodic RTOS timer for pwm_no_pulse_check_cb()
    osTimerDef_t pwm_check_callback_struct;
    pwm_check_callback_struct.ptimer = &pwm_no_pulse_check_cb; // Callback function pointer
    osTimerId pwm_check_timer = osTimerCreate(&pwm_check_callback_struct, osTimerPeriodic, NULL);
    osTimerStart (pwm_check_timer, 5); // Period in milliseconds
}

void diff_steering_mixer_update(int gpio_num) {
    if (gpio_num != board_config.mixer_mapping.gpio_update_trigger)
        return;

    Endpoint* endpoint_a = get_endpoint(board_config.mixer_mapping.endpoint_output_a);
    Endpoint* endpoint_b = get_endpoint(board_config.mixer_mapping.endpoint_output_b);
    if (!endpoint_a || !endpoint_b)
        return;
    
    endpoint_a->set_from_float(board_config.mixer_mapping.direction_a * (board_config.mixer_mapping.input_throttle + board_config.mixer_mapping.input_steering));
    endpoint_b->set_from_float(board_config.mixer_mapping.direction_b * (board_config.mixer_mapping.input_throttle - board_config.mixer_mapping.input_steering));
}

float add_deadband(float ratio) {
    if (ratio < PWM_DEADBAND_MIN) { // Between minimum and inclusive deadband
        return 0.5f * (ratio / PWM_DEADBAND_MIN); // Value from 0.0 to 0.5
    }
    else if (ratio <= PWM_DEADBAND_MAX) { // Inside inclusive deadband
        return 0.5f;
    }
    else { // Between max and min
        return 0.5f * (ratio - PWM_DEADBAND_MAX) / (1.0f - PWM_DEADBAND_MAX) + 0.5f; // Value from 0.5 to 1.0
    }
}

bool get_dir_pin_state(int32_t gpio_index) {
    int32_t gpio_dir_pin = board_config.pwm_mappings[gpio_index].gpio_direction_pin;

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (gpio_dir_pin >= 1 && gpio_dir_pin <= PWM_IN_COUNT) { // 1-indexed
#else
    // Only ch4 is available on v3.2
    if (gpio_dir_pin == 4) {
#endif
        return last_pin_state[gpio_dir_pin-1];
    }
    else {
        return true;
    }
}

void set_channel_output(Endpoint* endpoint, int32_t gpio_index, float ratio) {
    channel_value[gpio_index] = board_config.pwm_mappings[gpio_index].min +
                            (ratio * (board_config.pwm_mappings[gpio_index].max - board_config.pwm_mappings[gpio_index].min));

    bool dir_pin_state = get_dir_pin_state(gpio_index);

    endpoint->set_from_float((dir_pin_state) ? channel_value[gpio_index] : -channel_value[gpio_index]);

    diff_steering_mixer_update(gpio_index+1); // User-facing GPIO numbers are 1-indexed
}


/**
 * @brief Interpret RC or duty-cycle PWM into a value between min and max according to the pwm_mappings settings.
 * 
 * @param gpio_index The zero-indexed GPIO pin.  GPIO pin 1 is index 0.
 * @param high_time Time between LtoH and HtoL transitions
 * @param period Time between LtoH transitions
 * @return true When a valid pulse is interpreted or no endpoint is set
 * @return false When a pulse is invalid
 */
bool handle_pulse(int32_t gpio_index, Endpoint* endpoint, uint32_t high_time, uint32_t period) {
    float ratio;
    switch(board_config.pwm_mappings[gpio_index].pwm_type) {
        case PWM_TYPE_RC: {
            if (high_time < RC_MIN_LEGAL_HIGH_TIME || high_time > RC_MAX_LEGAL_HIGH_TIME)
                return false;
            ratio = (float)(high_time - RC_MIN_HIGH_TIME) / (float)(RC_MAX_HIGH_TIME - RC_MIN_HIGH_TIME);
            ratio = (ratio > 1.0f) ? 1.0f : ((ratio < 0.0f) ? 0.0f : ratio); // Limit to [0.0, 1.0]
        } break;
        case PWM_TYPE_DUTY_CYCLE: {
            if (high_time > period)
                return false;
            ratio = (float)high_time / (float)period;
        } break;
        default: {
            return true;
        } break;
    }

    if (board_config.pwm_mappings[gpio_index].enable_deadband) {
        ratio = add_deadband(ratio);
    }

    set_channel_output(endpoint, gpio_index, ratio);

    return true;
}

// In this function we need to calculate the duty cycle of the incoming PWM signal so that it can be translated into a control input

// Three types of pulse-width modulation (PWM) are possible:
// 1. The pulse center may be fixed in the center of the time window and both edges of the pulse moved to compress or expand the width.
// 2. The lead edge can be held at the lead edge of the window and the tail edge modulated.
// 3. The tail edge can be fixed and the lead edge modulated.

// Is there a good way to check the period of all 3 types?  If not, type 2 should be the most common in this situation.
// Type 2 is also the RC PWM signal type, so this implementation will work for both.

// How can we verify that a pulse is a valid signal? RC PWM has this built-in, where a pulse needs to be within a certain length, and the period should be around
// 20ms, but duty-cycle PWM has no such restrictions.  Right after boot, the old code waits for several HtoL transitions before it considers the input valid, but
// shouldn't this be done every time a signal goes inactive?
// We could compare the period of each pulse to the previous one, and require one or more consecutive matches for the pulse to be considered valid.  This would
// work for RC PWM too, and would replace the previous validation method.

// dutyCycle = highTime / period

// Calculating the period with either of these two formulas...
// period = highTime + lowTime
// period = HtoL_timestamp - last_HtoL_timestamp
// ...depends on knowing the order of the high/low pulses in the cycle.  If we assume it's type two, it will be high then low.
// As long as the duty cycle changes relatively slowly, if it's type 1 or 3 it shouldn't be too inaccurate.

// We also need to handle the case where the PWM signal has no duty cycle, and is simply switching between max and min at random times.
// We need to put a minimum period requirement on the signal, because there is no other way to differentiate between, say, switching from high to low
// every 3 seconds, and a 50% PWM signal with a period of 6 seconds.

// Timer CCxOF bit can be checked to see if two captures occurred, the second overwriting the first
// "In order to handle the overcapture, it is recommended to read the data before the
//  overcapture flag. This is to avoid missing an overcapture which could happen after reading
//  the flag and before reading the data."
// Maybe we can use this to set the output values to min or max depending on current pin state

// Channel inactive status is used differently depending on the PWM type.
// RC PWM:
//   Sets channel to (min+max)/2
// Duty-cycle PWM:
//   Differentiates between valid duty-cycle pulses and random switches between min and max output.
//   Basically, at some frequency, transitions between high and low are too slow to interpret as valid
//   duty-cycle PWM pulses, and must instead be interpreted as commands for min and max output.  This frequency cutoff
//   needs to be high enough for the min and max commands to be responsive.

// Called by interrupt from stm32f4xx.c
void pwm_in_cb(int channel, uint32_t timestamp) {
    // gpio_num is 1-indexed
    int gpio_num = tim_2_5_channel_num_to_gpio_num(channel);
    if (gpio_num <= 0) // Returns -1 for error
        return;

    bool current_pin_state = HAL_GPIO_ReadPin(get_gpio_port_by_pin(gpio_num), get_gpio_pin_by_pin(gpio_num)) != GPIO_PIN_RESET;

    int gpio_index = gpio_num - 1; // Convert from 1-indexed to zero-indexed for array access

    Endpoint* endpoint = get_endpoint(board_config.pwm_mappings[gpio_index].endpoint);
        
    auto transitionLtoH = [&]() {
        // Calculate LtoH period of waveform (assuming type 2 PWM)
        uint32_t this_period = timestamp - last_LtoH_timestamp[gpio_index];

        // We are calculating duty cycle at the LtoH pulse because that should be the end of the previous cycle (assuming type 2)
        // Ignore this PWM cycle if it doesn't match the period of the previous cycle within 10% or if the period is too long
        if ((this_period >= last_period_090[gpio_index]) && (this_period <= last_period_110[gpio_index]) && this_period < PWM_INACTIVE_TIME) {
            if (handle_pulse(gpio_index, endpoint, last_HtoL_timestamp[gpio_index] - last_LtoH_timestamp[gpio_index], this_period)) {
                if (board_config.pwm_mappings[gpio_index].pwm_type == PWM_TYPE_RC) {
                    last_active_timestamp[gpio_index] = timestamp; // Used for RC PWM
                    channel_active[gpio_index] = true;
                }
            }
            valid_counts[gpio_index]++;
        }

        last_LtoH_timestamp[gpio_index] = timestamp;
        last_period_110[gpio_index] = 11 * (this_period / 10); // Going to accept less accuracy here to avoid overflowing at (uint32_t max value)/10
        last_period_090[gpio_index] =  9 * (this_period / 10); // Going to accept less accuracy here to avoid overflowing at (uint32_t max value)/10
        last_period[gpio_index] = this_period;

        all_counts[gpio_index]++;
    };

    auto transitionHtoL = [&]() {
        last_HtoL_timestamp[gpio_index] = timestamp;
    };

    if (endpoint) {
        // If just transitioned from high to low (HtoL)
        if ((last_pin_state[gpio_index] == PWM_HIGH_STATE) && (current_pin_state != PWM_HIGH_STATE)) {
            transitionHtoL();
        }
        // If just transitioned from low to high (LtoH)
        else if ((last_pin_state[gpio_index] != PWM_HIGH_STATE) && (current_pin_state == PWM_HIGH_STATE)) {
            transitionLtoH();
        }
        else {
            // The pin state from the last interrupt is the same as the current pin state.  This should only occur when a pulse
            // is too short for an interrupt to occur for each edge, and we only caught the last edge.
            // In this case, since we can't measure the pulse length, it will be assumed to be max or min, depending on the current state.

            if (current_pin_state == PWM_HIGH_STATE) { // HtoL then LtoH
                // Immeasurably-short low pulse
                // Duty cycle is 100%
                transitionHtoL();
                transitionLtoH();
            }
            else { // LtoH then HtoL 
                // Immeasurably-short high pulse
                // Duty cycle is 0%
                transitionLtoH();
                transitionHtoL();
            }
        }
    }
    else {
        static uint32_t counter = 0;
        printf("%u\n", ++counter);
        for (size_t i = 0; i < PWM_IN_COUNT; ++i) {
            // Check if another GPIO pin is using this channel as direction pin
            if (board_config.pwm_mappings[i].gpio_direction_pin == gpio_num) {
                // If so, apply direction pin state change to that channel
                Endpoint* i_endpoint = get_endpoint(board_config.pwm_mappings[i].endpoint);
                if (i_endpoint) {
                    i_endpoint->set_from_float((current_pin_state) ? channel_value[i] : -channel_value[i]);
                    diff_steering_mixer_update(i+1); // User-facing GPIO numbers are 1-indexed
                }
            }
        }
    }

    last_pin_state[gpio_index] = current_pin_state;

    if (board_config.pwm_mappings[gpio_index].pwm_type == PWM_TYPE_DUTY_CYCLE) {
        last_active_timestamp[gpio_index] = timestamp;
        channel_active[gpio_index] = true;
    }
}

// Called by RTOS timer at 0.1 to 1 kHz.  Timer set up in pwm_in_init()
void pwm_no_pulse_check_cb(void *argument) {
    
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    for (uint8_t gpio_index = 0; gpio_index < PWM_IN_COUNT; gpio_index++) {
#else
    // Only ch4 is available on v3.2
    uint8_t gpio_index = 4; {
#endif
        // Only check channel if endpoint is assigned
        Endpoint* endpoint = get_endpoint(board_config.pwm_mappings[gpio_index].endpoint);
        if (endpoint && channel_active[gpio_index]) {

            // 32-bit timer register overflows in 51 seconds at 84MHz

            // RC PWM:
            //   If time_since_valid_pulse > PWM_INACTIVE_TIME, set channel inactive and set output to center (assumed to be the safe, disabled option)
            //   The interrupt function will set the channel active again if a valid pulse is received
            // Duty-cycle PWM:
            //   If the time since an edge exceeds PWM_INACTIVE_TIME, set channel inactive and set output to min or max depending on last_pin_state
            //   Poll direction pin and set output correspondingly, because a change in that pin doesn't trigger an interrupt

            switch(board_config.pwm_mappings[gpio_index].pwm_type) {
                case PWM_TYPE_RC: {
                    if ((htim5.Instance->CNT - last_active_timestamp[gpio_index]) > PWM_INACTIVE_TIME) {
                        channel_active[gpio_index] = false;
                        set_channel_output(endpoint, gpio_index, 0.5);
                        printf("RC input index %i timed out\n", gpio_index);
                    }
                } break;
                case PWM_TYPE_DUTY_CYCLE: {
                    if ((htim5.Instance->CNT - last_active_timestamp[gpio_index]) > PWM_INACTIVE_TIME) {
                        channel_active[gpio_index] = false;

                        // Interrupt always records the most recent pin state in last_pin_state[gpio_index]
                        if (last_pin_state[gpio_index] == PWM_HIGH_STATE) {
                            set_channel_output(endpoint, gpio_index, 1.0);
                        }
                        else {
                            set_channel_output(endpoint, gpio_index, 0.0);
                        }

                        printf("PWM input index %i timed out\n", gpio_index);
                    }
                } break;
            }
        }
    }
}