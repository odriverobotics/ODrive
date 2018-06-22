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

#if HW_VERSION_MAJOR == 3
#if HW_VERSION_MINOR <= 3
#define SHUNT_RESISTANCE (675e-6f)
#else
#define SHUNT_RESISTANCE (500e-6f)
#endif
#endif


typedef struct {
    GPIO_TypeDef* step_port;
    uint16_t step_pin;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
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
} EncoderHardwareConfig_t;
typedef struct {
    TIM_HandleTypeDef* timer;
    uint16_t control_deadline;
    float shunt_conductance;
} MotorHardwareConfig_t;
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
    MotorHardwareConfig_t motor_config;
    GateDriverHardwareConfig_t gate_driver_config;
} BoardHardwareConfig_t;

extern const BoardHardwareConfig_t hw_configs[2];

//TODO stick this in a C file
#ifdef __MAIN_CPP__
const BoardHardwareConfig_t hw_configs[2] = { {
    .axis_config = {
        .step_port = GPIO_1_GPIO_Port,
        .step_pin = GPIO_1_Pin,
        .dir_port = GPIO_2_GPIO_Port,
        .dir_pin = GPIO_2_Pin,
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
    },
    .motor_config = {
        .timer = &htim1,
        .control_deadline = TIM_1_8_PERIOD_CLOCKS,
        .shunt_conductance = 1.0f / SHUNT_RESISTANCE,  //[S]
    },
    .gate_driver_config = {
        .spi = &hspi3,
        // Note: this board has the EN_Gate pin shared!
        .enable_port = EN_GATE_GPIO_Port,
        .enable_pin = EN_GATE_Pin,
        .nCS_port = M0_nCS_GPIO_Port,
        .nCS_pin = M0_nCS_Pin,
        .nFAULT_port = nFAULT_GPIO_Port, // the nFAULT pin is shared between both motors
        .nFAULT_pin = nFAULT_Pin,
    }
},{
    .axis_config = {
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5
        .step_port = GPIO_7_GPIO_Port,
        .step_pin = GPIO_7_Pin,
        .dir_port = GPIO_8_GPIO_Port,
        .dir_pin = GPIO_8_Pin,
#else
        .step_port = GPIO_3_GPIO_Port,
        .step_pin = GPIO_3_Pin,
        .dir_port = GPIO_4_GPIO_Port,
        .dir_pin = GPIO_4_Pin,
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
    },
    .motor_config = {
        .timer = &htim8,
        .control_deadline = (3 * TIM_1_8_PERIOD_CLOCKS) / 2,
        .shunt_conductance = 1.0f / SHUNT_RESISTANCE,  //[S]
    },
    .gate_driver_config = {
        .spi = &hspi3,
        // Note: this board has the EN_Gate pin shared!
        .enable_port = EN_GATE_GPIO_Port,
        .enable_pin = EN_GATE_Pin,
        .nCS_port = M1_nCS_GPIO_Port,
        .nCS_pin = M1_nCS_Pin,
        .nFAULT_port = nFAULT_GPIO_Port, // the nFAULT pin is shared between both motors
        .nFAULT_pin = nFAULT_Pin,
    }
} };
#endif



#define I2C_A0_PORT GPIO_3_GPIO_Port
#define I2C_A0_PIN GPIO_3_Pin
#define I2C_A1_PORT GPIO_4_GPIO_Port
#define I2C_A1_PIN GPIO_4_Pin
#define I2C_A2_PORT GPIO_5_GPIO_Port
#define I2C_A2_PIN GPIO_5_Pin

#endif // __BOARD_CONFIG_H
