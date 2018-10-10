
#ifndef _HAL_STRUCTS_H
#define _HAL_STRUCTS_H
#include "cmsis_os.h"
typedef struct {
    uint16_t step_gpio_pin;
    uint16_t dir_gpio_pin;
    size_t thermistor_adc_ch;
    osPriority thread_priority;
} AxisHardwareConfig_t;

typedef struct {
    TIM_HandleTypeDef *timer;
    GPIO_TypeDef *index_port;
    uint16_t index_pin;
    GPIO_TypeDef *hallA_port;
    uint16_t hallA_pin;
    GPIO_TypeDef *hallB_port;
    uint16_t hallB_pin;
    GPIO_TypeDef *hallC_port;
    uint16_t hallC_pin;
} EncoderHardwareConfig_t;

typedef struct {
    TIM_HandleTypeDef *timer;
    uint16_t control_deadline;
    float shunt_conductance;
} MotorHardwareConfig_t;

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    GPIO_TypeDef *nCS_port;
    uint16_t nCS_pin;
    GPIO_TypeDef *nFAULT_port;
    uint16_t nFAULT_pin;
} GateDriverHardwareConfig_t;

typedef struct {
    AxisHardwareConfig_t axis_config;
    EncoderHardwareConfig_t encoder_config;
    MotorHardwareConfig_t motor_config;
    GateDriverHardwareConfig_t gate_driver_config;
} BoardHardwareConfig_t;



#endif //_HAL_STRUCTS_H
