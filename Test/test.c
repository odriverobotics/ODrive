
#include "test.h"

#include "adc.h"
#include "tim.h"

#include "math.h"
#include "stdint.h"

/////////////////////////////////////////////////
//Test adc conversion latency and triggering

void test_adc_trigger() {
    //Set trigger to mid phase to check for trigger polarity
    htim1.Instance->CCR4 = 2048;

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
    __HAL_ADC_ENABLE(&hadc2);

    //Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
}

static int test = 0;
static int test2 = 0;
static uint32_t testcnt[16];
static int tcidx = 0;
void test_adc_trigger_cb() {
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
/////////////////////////////////////////////////

static int cbcnt = 0;
void test_cb_count(){
    ++cbcnt;
}


/////////////////////////////////////////////////
//Histogram test
static float alpha = 1/(5000.0f);
static float avg = 2048.0f;
static float var = 0.0f;
static uint32_t hist_countdown = 40000;
static uint32_t errhist[20];
static uint32_t neg_errhist[20];
void test_adc_hist_cb(ADC_HandleTypeDef* hadc) {
    //float unknown_ch_volts = read_ADC_volts(hadc, 1);

    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, 1);
    float val = (float)ADCValue;
    avg *= (1.0f - alpha);
    avg += alpha * val;

    float dval = val-avg;
    var *= (1.0f - alpha);
    var += alpha * (dval * dval);

    if (hist_countdown) {
        --hist_countdown;
    } else {
        int idval = (int)dval;
        int pos = (idval >= 0);
        if (!pos)
            idval = -idval;
        if (idval >= 20)
            idval = 19;
        if (pos)
            ++errhist[idval];
        else
            ++neg_errhist[idval];
    }
}
/////////////////////////////////////////////////


//Test setup: Setup tests in main, and set callbacks
void test_main(void) {

    //test_adc_trigger();

}

//Uncomment to define the ADC callback. Ordinarily it is used by motor control
// void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
//     //test_adc_trigger_cb(hadc);
//     //test_cb_count();
//     //test_adc_hist_cb(hadc);
//     test_pwm_from_adc_cb(hadc);
// }

