
#include "test.h"

#include "stm32f4xx_hal.h"
#include "stm32f405xx.h"
#include "assert.h"
#include "cmsis_os.h"

#include "adc.h"
#include "tim.h"
#include "spi.h"
#include "drv8301.h"

#include "math.h"
#include "stdint.h"

static void test_read_ADC(void);

static int test_base = 0;
static int test = 0;
static int test2 = 0;

void start_DRV8301() {

    //Init PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

    //Turn off output
    __HAL_TIM_MOE_DISABLE(&htim1);

    //Enable driver
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);

    //Wait for startup
    osDelay(10);

    //Do SPI comms
    HAL_GPIO_WritePin(M0_nCS_GPIO_Port, M0_nCS_Pin, GPIO_PIN_RESET);
    osDelay(1);

    uint16_t zerobuff = 0;
    uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Read, DRV8301_RegName_Status_2, 0);
    uint16_t recbuff = 0xbeef;
    HAL_SPI_Transmit(&hspi3, &controlword, 1, 1000);
    HAL_SPI_TransmitReceive(&hspi3, &zerobuff, &recbuff, 1, 1000);
    osDelay(1);
    HAL_GPIO_WritePin(M0_nCS_GPIO_Port, M0_nCS_Pin, GPIO_PIN_SET);
    osDelay(1);


}

void test_adc_trigger() {
    //Set trigger to mid phase to check for trigger polarity
    htim1.Instance->CCR4 = 2048;

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
    __HAL_ADC_ENABLE(&hadc2);

    //Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
}

void DRV8301_setup() {

}

void test_main(void) {

    //start_DRV8301();
    DRV8301_setup();
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

