
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

void start_adc_pwm(){

    //Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    //Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOC);

    //Init PWM
    int half_load = htim1.Instance->ARR/2;
    htim1.Instance->CCR1 = half_load;
    htim1.Instance->CCR2 = half_load;
    htim1.Instance->CCR3 = half_load;

    //This hardware obfustication layer really is getting on my nerves
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    htim1.Instance->CCR4 = 1;
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

    //Turn off output
    //__HAL_TIM_MOE_DISABLE(&htim1);

}

typedef struct Motor_s {
    DRV8301_Obj gate_driver;
    float shunt_conductance;
    float maxcurrent;
} Motor_t;

Motor_t motor_configs[] = {
    { //M0
        .gate_driver = {
            .spiHandle = &hspi3,
            //Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M0_nCS_GPIO_Port,
            .nCSgpioNumber = M0_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false
        },
        .shunt_conductance = 1.0f/0.0005f, //[S]
        .maxcurrent = 75.0f //[A] //Note: consistent with 40v/v gain
    }
};

static const int num_motors = sizeof(motor_configs)/sizeof(motor_configs[0]);

//Local view of DRV registers
static DRV_SPI_8301_Vars_t gate_driver_regs[1/*num_motors*/];

void test_DRV8301_setup() {
    for (int i = 0; i < num_motors; ++i) {
        DRV8301_enable(&motor_configs[i].gate_driver);
        DRV8301_setupSpi(&motor_configs[i].gate_driver, &gate_driver_regs[i]);

        //@TODO we can use reporting only if we actually wire up the nOCTW pin
        gate_driver_regs[i].Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
        //Overcurrent set to approximately 150A at 100degC. This may need tweaking.
        gate_driver_regs[i].Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
        //20V/V on 500uOhm gives a range of +/- 150A
        //40V/V on 500uOhm gives a range of +/- 75A
        gate_driver_regs[i].Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_40VpV;

        gate_driver_regs[i].SndCmd = true;
        DRV8301_writeData(&motor_configs[i].gate_driver, &gate_driver_regs[i]);
        gate_driver_regs[i].RcvCmd = true;
        DRV8301_readData(&motor_configs[i].gate_driver, &gate_driver_regs[i]);
    }
}

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

float phase_current_from_adcval(uint32_t ADCValue, int motornum) {
    float rev_gain;
    switch (gate_driver_regs[motornum].Ctrl_Reg_2.GAIN) {
        case DRV8301_ShuntAmpGain_10VpV:
            rev_gain = 1.0f/10.0f;
            break;
        case DRV8301_ShuntAmpGain_20VpV:
            rev_gain = 1.0f/20.0f;
            break;
        case DRV8301_ShuntAmpGain_40VpV:
            rev_gain = 1.0f/40.0f;
            break;
        case DRV8301_ShuntAmpGain_80VpV:
            rev_gain = 1.0f/80.0f;
            break;
    }

    int adcval_bal = (int)ADCValue - (1<<11);
    float amp_out_volt = (3.3f/(float)(1<<12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * rev_gain;
    float current = shunt_volt * motor_configs[motornum].shunt_conductance;
    return current;
}

void assertt(int arg) {
    if(!arg) {
        for(;;);
    }
}

// current sense queue from ADC to motor control task
typedef struct {
    float current_phB;
    float current_phC;
} Iph_BC_queue_item_t;
osMailQDef (Iph_queue_def, 2, Iph_BC_queue_item_t);
osMailQId  (M0_Iph_queue);

void test_pwm_from_adc_cb(ADC_HandleTypeDef* hadc) {

    // ADC2 and ADC3 record the phB and phC currents concurrently,
    // and their interrupts should arrive on the same clock cycle.
    // The HAL issues the callbacks in order, so ADC2 will always be processed before ADC3.
    // Therefore we store the value from ADC2 and push them both into the queue
    // when ADC3 is ready.
    // @TODO: don't use statics, will only work for 1 motor chanel
    static float phB_current;

    //Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);

    //Store and return, or fetch and continue
    float phC_current;
    if (hadc == &hadc2) {
        phB_current = phase_current_from_adcval(ADCValue, 0);
        return;
    } else if (hadc == &hadc3) {
        phC_current = phase_current_from_adcval(ADCValue, 0);
    } else {
        //hadc is something else, not expected
        assertt(0);
    }

    //Allocate mail queue storage
    Iph_BC_queue_item_t* mail_ptr;
    mail_ptr = (Iph_BC_queue_item_t*) osMailAlloc(M0_Iph_queue, 0);
    if (mail_ptr == NULL) {
        return;
    }

    //Write contents and send mail
    mail_ptr->current_phB = phB_current;
    mail_ptr->current_phC = phC_current;
    osMailPut(M0_Iph_queue, mail_ptr);

}


void test_motor_thread(void const * argument) {

    //Allocate the queues
    M0_Iph_queue = osMailCreate(osMailQ(Iph_queue_def), NULL);

    //Init gate drivers
    test_DRV8301_setup();

    osDelay(1000);

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    for(;;) {
        //Current measurements not occurring in a timely manner can be handled by the watchdog
        //@TODO Actually make watchdog
        //Hence we can use osWaitForever
        osEvent evt = osMailGet(M0_Iph_queue, osWaitForever);

        //Since we wait forever, we do not expect timeouts here.
        assertt(evt.status == osEventMail);

        Iph_BC_queue_item_t* mail_ptr = evt.value.p;
        float M0_phB_current = mail_ptr->current_phB;
        float M0_phC_current = mail_ptr->current_phC;
        osMailFree(M0_Iph_queue, mail_ptr);

        // WARNING: Only gimbal motors!!
        int half_load = htim1.Instance->ARR/2;
        htim1.Instance->CCR1 = half_load - 400;
        htim1.Instance->CCR2 = half_load + 400;
        htim1.Instance->CCR3 = half_load + 400;
    }
}


//Test setup: Setup tests in main, and set callbacks
void test_main(void) {

    //test_adc_trigger();

}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    //test_adc_trigger_cb(hadc);
    //test_cb_count();
    //test_adc_hist_cb(hadc);
    test_pwm_from_adc_cb(hadc);
}

