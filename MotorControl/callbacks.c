#include "callbacks.h"
#include "constants.h"

void vbus_sense_adc_cb(ADC_HandleTypeDef *hadc, bool injected) {
    static const float voltage_scale = 3.3f * 11.0f / (float)(1 << 12);
    // Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    vbus_voltage = ADCValue * voltage_scale;
}

// step/direction interface
void step_cb(uint16_t GPIO_Pin) {
    GPIO_PinState dir_pin;
    float dir;
    switch (GPIO_Pin) {
        case GPIO_1_Pin:
            // M0 stepped
            if (motors[0].enable_step_dir) {
                dir_pin = HAL_GPIO_ReadPin(GPIO_2_GPIO_Port, GPIO_2_Pin);
                dir = (dir_pin == GPIO_PIN_SET) ? 1.0f : -1.0f;
                motors[0].pos_setpoint += dir * motors[0].counts_per_step;
            }
            break;
        case GPIO_3_Pin:
            // M1 stepped
            if (motors[1].enable_step_dir) {
                dir_pin = HAL_GPIO_ReadPin(GPIO_4_GPIO_Port, GPIO_4_Pin);
                dir = (dir_pin == GPIO_PIN_SET) ? 1.0f : -1.0f;
                motors[1].pos_setpoint += dir * motors[1].counts_per_step;
            }
            break;
        default:
            global_fault(ERROR_UNEXPECTED_STEP_SRC);
            break;
    }
}