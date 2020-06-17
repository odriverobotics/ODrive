/*
* @brief Contains board specific configuration for ODrive v3.x
*/

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

// STM specific includes
#include <gpio.h>
#include <spi.h>
#include <tim.h>
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


static const size_t AXIS_COUNT = 2;


#ifdef __cplusplus
#include <Drivers/STM32/stm32_gpio.hpp>
#include <Drivers/DRV8301/drv8301.hpp>
#include <MotorControl/thermistor.hpp>

using TGateDriver = Drv8301;
using TOpAmp = Drv8301;

#include <MotorControl/motor.hpp>

extern Motor m0;
extern Motor m1;
extern OnboardThermistorCurrentLimiter m0_fet_thermistor;
extern OnboardThermistorCurrentLimiter m1_fet_thermistor;
extern Motor* motors[AXIS_COUNT];
extern OnboardThermistorCurrentLimiter* fet_thermistors[AXIS_COUNT];

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
    TIM_HandleTypeDef* timer;
    GPIO_TypeDef* index_port;
    uint16_t index_pin;
    GPIO_TypeDef* hallA_port;
    uint16_t hallA_pin;
    GPIO_TypeDef* hallB_port;
    uint16_t hallB_pin;
    GPIO_TypeDef* hallC_port;
    uint16_t hallC_pin;
    SPI_HandleTypeDef* spi;
} EncoderHardwareConfig_t;
typedef struct {
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* enable_port;
    uint16_t enable_pin;
    GPIO_TypeDef* nCS_port;
    uint16_t nCS_pin;
    GPIO_TypeDef* nFAULT_port;
    uint16_t nFAULT_pin;
} GateDriverHardwareConfig_t;
typedef struct {
    AxisHardwareConfig_t axis_config;
    EncoderHardwareConfig_t encoder_config;
} BoardHardwareConfig_t;

extern const BoardHardwareConfig_t hw_configs[2];

//TODO stick this in a C file
#ifdef __MAIN_CPP__

const BoardHardwareConfig_t hw_configs[2] = { {
    //M0
    .axis_config = {
        .step_gpio_pin = 1,
        .dir_gpio_pin = 2,
        .thread_priority = (osPriority)(osPriorityHigh + (osPriority)1),
    },
    .encoder_config = {
        .timer = &htim3,
        .index_port = M0_ENC_Z_GPIO_Port,
        .index_pin = M0_ENC_Z_Pin,
        .hallA_port = M0_ENC_A_GPIO_Port,
        .hallA_pin = M0_ENC_A_Pin,
        .hallB_port = M0_ENC_B_GPIO_Port,
        .hallB_pin = M0_ENC_B_Pin,
        .hallC_port = M0_ENC_Z_GPIO_Port,
        .hallC_pin = M0_ENC_Z_Pin,
        .spi = &hspi3,
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
    .encoder_config = {
        .timer = &htim4,
        .index_port = M1_ENC_Z_GPIO_Port,
        .index_pin = M1_ENC_Z_Pin,
        .hallA_port = M1_ENC_A_GPIO_Port,
        .hallA_pin = M1_ENC_A_Pin,
        .hallB_port = M1_ENC_B_GPIO_Port,
        .hallB_pin = M1_ENC_B_Pin,
        .hallC_port = M1_ENC_Z_GPIO_Port,
        .hallC_pin = M1_ENC_Z_Pin,
        .spi = &hspi3,
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


#endif // __BOARD_CONFIG_H
