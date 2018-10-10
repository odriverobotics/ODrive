/*
* @brief Contains board specific configuration for ODrive v3.x
*/

#ifndef __BOARD_CONFIG_H_V3
#define __BOARD_CONFIG_H_V3

// STM specific includes
#include <gpio.h>
#include <spi.h>
#include <tim.h>
#include <main.h>

#if HW_VERSION_MINOR <= 3
    #define M0_SHUNT_RESISTANCE (675e-6f)
    #define M1_SHUNT_RESISTANCE (675e-6f)
#else
    #define M0_SHUNT_RESISTANCE (500e-6f)
    #define M1_SHUNT_RESISTANCE (500e-6f)
#endif


#define THERMISTOR_POLY_COEFFS {363.93910201f, -462.15369634f, 307.55129571f, -27.72569531f}

#define HAL_NUMBER_OF_MOTORS 2

#define M0_AXIS_STEP_PIN 1
#define M0_AXIS_DIR_PIN 2
#define M0_THERMISTOR_ADC_CHANNEL 15

#define M0_THREAD_PRIORITY_OFFSET 1


#if HW_VERSION_MINOR >= 5
    #define M1_AXIS_STEP_PIN 7
    #define M1_AXIS_DIR_PIN 8
#else
    #define M1_AXIS_STEP_PIN 3
    #define M1_AXIS_DIR_PIN 4
#endif

#if HW_VERSION_MINOR >= 3
    #define M1_ADC_CHANNEL 4
#else
    #define M1_ADC_CHANNEL 1
#endif

#define M1_THERMISTOR_ADC_CHANNEL 15
#define M1_THREAD_PRIORITY_OFFSET 0

#if HW_VERSION_MINOR >= 5 && HW_VERSION_VOLTAGE >= 48
    #define HAL_BRAKE_RESISTANCE 2.0f     // [ohm]
#else
    #define HAL_BRAKE_RESISTANCE 0.47f;     // [ohm]
#endif


#define HAL_I2C_A0_PORT GPIO_3_GPIO_Port
#define HAL_I2C_A0_PIN GPIO_3_Pin
#define HAL_I2C_A1_PORT GPIO_4_GPIO_Port
#define HAL_I2C_A1_PIN GPIO_4_Pin
#define HAL_I2C_A2_PORT GPIO_5_GPIO_Port
#define HAL_I2C_A2_PIN GPIO_5_Pin

#endif // __BOARD_CONFIG_H_V3
