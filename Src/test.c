
#include "test.h"

#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_adc.h"
#include "adc.h"

static void test_read_ADC(void);

void test_main(void) {
    test_read_ADC();
}

// Simple loop to read ADC1
static void test_read_ADC(void){
    int measnum = 0;
    for (;;) {
        HAL_ADC_Start(&hadc1);
        osDelay(2);
        if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
            uint32_t ADCValue = HAL_ADC_GetValue(&hadc1);
            ++measnum;
        }
        osDelay(1000);
    }
}
