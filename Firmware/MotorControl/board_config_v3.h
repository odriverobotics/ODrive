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


struct AxisHardwareConfig_t {
    GPIO_TypeDef* step_port;
    uint16_t step_pin;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    osPriority thread_priority;
};

struct EncoderHardwareConfig_t {
    TIM_HandleTypeDef* timer;
    GPIO_TypeDef* index_port;
    uint16_t index_pin;
};
struct MotorHardwareConfig_t {
    TIM_HandleTypeDef* timer;
    uint16_t control_deadline;
    float shunt_conductance;
};
struct GateDriverHardwareConfig_t {
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* enable_port;
    uint16_t enable_pin;
    GPIO_TypeDef* nCS_port;
    uint16_t nCS_pin;
    GPIO_TypeDef* nFAULT_port;
    uint16_t nFAULT_pin;
};
struct BoardHardwareConfig_t {
    AxisHardwareConfig_t axis_config;
    EncoderHardwareConfig_t encoder_config;
    MotorHardwareConfig_t motor_config;
    GateDriverHardwareConfig_t gate_driver_config;
};

const BoardHardwareConfig_t hw_configs[] = { {
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
        .step_port = GPIO_3_GPIO_Port,
        .step_pin = GPIO_3_Pin,
        .dir_port = GPIO_4_GPIO_Port,
        .dir_pin = GPIO_4_Pin,
        .thread_priority = osPriorityHigh,
    },
    .encoder_config = {
        .timer = &htim4,
        .index_port = M1_ENC_Z_GPIO_Port,
        .index_pin = M1_ENC_Z_Pin,
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

#endif // __BOARD_CONFIG_H
