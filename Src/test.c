
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
            float vbus_s = read_ADC_volts(&hadc1);
            float busvoltage = (1.0f + 10.0f)/1.0f * vbus_s;
            ++measnum;
        }
        osDelay(10);
    }
}
