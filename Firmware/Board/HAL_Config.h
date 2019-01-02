#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H


#include "HAL_Generics/HAL_Structs.h"





#if HW_VERSION_MAJOR == 3
#include "Board/v3/HAL_Board_V3.x.h"
#else
#error "unknown board version"
#endif

//TODO stick this in a C file
#ifdef __MAIN_CPP__
const float thermistor_poly_coeffs[] = THERMISTOR_POLY_COEFFS;
const size_t thermistor_num_coeffs = sizeof(thermistor_poly_coeffs)/sizeof(thermistor_poly_coeffs[1]);

const BoardHardwareConfig_t hw_configs[HAL_NUMBER_OF_MOTORS] = { {
    //M0
    .axis_config = {
        .step_gpio_pin = M0_AXIS_STEP_PIN,
        .dir_gpio_pin = M0_AXIS_DIR_PIN,
        .thermistor_adc_ch = M0_THERMISTOR_ADC_CHANNEL,
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
    //M1
    .axis_config = {
        .step_gpio_pin = M1_AXIS_STEP_PIN,
        .dir_gpio_pin = M1_AXIS_DIR_PIN,
        .thermistor_adc_ch = M1_THERMISTOR_ADC_CHANNEL,
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

#endif //BOARD_CONFIG_H
