
#include "test.h"

#include "cmsis_os.h"
#include "stm32f405xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"
#include "assert.h"

#include "adc.h"
#include "tim.h"

#include "math.h"

static void test_read_ADC(void);

static int test_base = 0;
static int test = 0;
static int test2 = 0;

void test_main(void) {

    //Set trigger to mid phase to check for trigger polarity
    htim1.Instance->CCR4 = 2048;

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
    __HAL_ADC_ENABLE(&hadc2);

    //Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);

}

// Simple loop to read ADC1
static void test_read_ADC(void) {
    int measnum = 0;
    for (;;) {
        HAL_ADC_Start(&hadc1);
        osDelay(2);
        if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
            float vbus_s = read_ADC_volts(&hadc1, 0);
            float busvoltage = (1.0f + 10.0f)/1.0f * vbus_s;
            ++measnum;
        }
        osDelay(10);
    }
}

static uint32_t testcnt[16];
static int tcidx = 0;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    assert(hadc == &hadc2);

    float unknown_ch_volts = read_ADC_volts(hadc, 1);
    uint32_t cnt = htim1.Instance->CNT;
    int dir = htim1.Instance->CR1 & TIM_CR1_DIR;
    if(dir){
        test++;
    } else {
        test2++;
        testcnt[tcidx] = cnt - 2048;
        if(++tcidx == 16)
            tcidx = 0;
    }
}

