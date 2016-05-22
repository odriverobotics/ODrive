
#include "test.h"

#include "stm32f4xx_hal_adc.h"
#include "adc.h"

static void test_read_ADC(void);

void test_main(void) {
    test_read_ADC();
}

// Simple loop to read ADC1
static void test_read_ADC(void){
    HAL_ADC_Start(&hadc1);

}
