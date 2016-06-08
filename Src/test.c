
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

void start_pwm_triggering(){
    //Init PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

    //Turn off output
    __HAL_TIM_MOE_DISABLE(&htim1);

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

DRV8301_Obj gate_drivers[] = {
    {
        //M0

        .spiHandle = &hspi3,
        //Note: this board has the EN_Gate pin shared!
        .EngpioHandle = EN_GATE_GPIO_Port,
        .EngpioNumber = EN_GATE_Pin,
        .nCSgpioHandle = M0_nCS_GPIO_Port,
        .nCSgpioNumber = M0_nCS_Pin,
        .RxTimeOut = false,
        .enableTimeOut = false
    }
};
static const int num_motors = sizeof(gate_drivers)/sizeof(gate_drivers[0]);

void test_DRV8301_setup() {

    //Local view of DRV registers
    DRV_SPI_8301_Vars_t gate_driver_regs[num_motors];

    //The DRV_8301 driver instance
    //DRV8301_Obj gate_drivers[NUM_MOTORS];

    for (int i = 0; i < num_motors; ++i) {
        DRV8301_enable(&gate_drivers[i]);
        DRV8301_setupSpi(&gate_drivers[i], &gate_driver_regs[i]);

        //@TODO we can use reporting only if we actually wire up the nOCTW pin
        gate_driver_regs[i].Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
        //Overcurrent set to approximately 150A at 100degC. This may need tweaking.
        gate_driver_regs[i].Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
        //20V/V on 500uOhm gives a range of +/- 150A
        gate_driver_regs[i].Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_20VpV;

        gate_driver_regs[i].SndCmd = true;
        DRV8301_writeData(&gate_drivers[i], &gate_driver_regs[i]);
        gate_driver_regs[i].RcvCmd = true;
        DRV8301_readData(&gate_drivers[i], &gate_driver_regs[i]);

        osDelay(1000);
    }
}

void test_main(void) {

    //start_DRV8301();
    test_DRV8301_setup();
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

