/*
* @brief Contains board specific configuration for ODrive v3.x
*/

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H


// STM specific includes
#include <stm32f4xx_hal.h>
#include <gpio.h>
#include <spi.h>
#include <tim.h>
#include <can.h>
#include <i2c.h>
#include <main.h>
#include "cmsis_os.h"

#include <arm_math.h>

#if HW_VERSION_MAJOR == 3
#if HW_VERSION_MINOR <= 3
#define SHUNT_RESISTANCE (675e-6f)
#else
#define SHUNT_RESISTANCE (500e-6f)
#endif
#endif


#define AXIS_COUNT (2)

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR <= 4
#define GPIO_COUNT  (5)
#elif HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5
#define GPIO_COUNT  (8)
#endif


#ifdef __cplusplus
#include <Drivers/STM32/stm32_gpio.hpp>
#include <Drivers/DRV8301/drv8301.hpp>
#include <MotorControl/thermistor.hpp>

using TGateDriver = Drv8301;
using TOpAmp = Drv8301;

#include <MotorControl/motor.hpp>
#include <MotorControl/encoder.hpp>

extern Motor motors[AXIS_COUNT];
extern OnboardThermistorCurrentLimiter fet_thermistors[AXIS_COUNT];
extern Encoder encoders[AXIS_COUNT];
extern Stm32Gpio gpios[GPIO_COUNT];
extern uint32_t pwm_in_gpios[4];

#include <Drivers/STM32/stm32_spi_arbiter.hpp>
extern Stm32SpiArbiter& ext_spi_arbiter;
#endif

// Period in [s]
static const float current_meas_period = CURRENT_MEAS_PERIOD;

// Frequency in [Hz]
static const int current_meas_hz = CURRENT_MEAS_HZ;


typedef struct {
    uint16_t step_gpio_pin;
    uint16_t dir_gpio_pin;
    osPriority thread_priority;
} AxisHardwareConfig_t;

typedef struct {
    AxisHardwareConfig_t axis_config;
} BoardHardwareConfig_t;

extern const BoardHardwareConfig_t hw_configs[AXIS_COUNT];

//TODO stick this in a C file
#ifdef __MAIN_CPP__

const BoardHardwareConfig_t hw_configs[AXIS_COUNT] = { {
    //M0
    .axis_config = {
        .step_gpio_pin = 1,
        .dir_gpio_pin = 2,
        .thread_priority = (osPriority)(osPriorityHigh + (osPriority)1),
    },
},{
    //M1
    .axis_config = {
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5
        .step_gpio_pin = 7,
        .dir_gpio_pin = 8,
#else
        .step_gpio_pin = 3,
        .dir_gpio_pin = 4,
#endif
        .thread_priority = osPriorityHigh,
    },
} };
#endif

#define I2C_A0_PORT GPIO_3_GPIO_Port
#define I2C_A0_PIN GPIO_3_Pin
#define I2C_A1_PORT GPIO_4_GPIO_Port
#define I2C_A1_PIN GPIO_4_Pin
#define I2C_A2_PORT GPIO_5_GPIO_Port
#define I2C_A2_PIN GPIO_5_Pin

// This board has no board-specific user configurations
static inline bool board_pop_config() { return true; }
static inline bool board_push_config() { return true; }
static inline void board_clear_config() { }
static inline bool board_apply_config() { return true; }

void board_init();

#endif // __BOARD_CONFIG_H
