
#include <low_level.h>

#include <cmsis_os.h>

#include <main.h>
#include <adc.h>
#include <tim.h>
#include <spi.h>


// Global variables
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
const int num_motors = sizeof(motor_configs)/sizeof(motor_configs[0]);

// Private variables
//Local view of DRV registers
static DRV_SPI_8301_Vars_t gate_driver_regs[1/*num_motors*/];

// current sense queue from ADC to motor control task
typedef struct {
    float current_phB;
    float current_phC;
} Iph_BC_queue_item_t;
osMailQDef (Iph_queue_def, 2, Iph_BC_queue_item_t);
osMailQId  (M0_Iph_queue);

// Private function prototypes
static void DRV8301_setup();
static void start_adc_pwm();
static float phase_current_from_adcval(uint32_t ADCValue, int motornum);
static void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc);


//Special function name for ADC callback.
//Automatically registered if defined.
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    pwm_trig_adc_cb(hadc);
}


void init_motor_control() {
    //Allocate the queues
    M0_Iph_queue = osMailCreate(osMailQ(Iph_queue_def), NULL);

    //Init gate drivers
    DRV8301_setup();

    osDelay(1000);

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();
}

// Set up the gate drivers
static void DRV8301_setup() {
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

static void start_adc_pwm(){
    //Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    //Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOC);

    //Ensure that debug halting of the core doesn't leave the motor PWM running
    __HAL_DBGMCU_FREEZE_TIM1();
    __HAL_DBGMCU_FREEZE_TIM8();

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

static float phase_current_from_adcval(uint32_t ADCValue, int motornum) {
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

//@TODO make available from anywhere
void safe_assert(int arg) {
    if(!arg) {
        __HAL_TIM_MOE_DISABLE(&htim1);
        __HAL_TIM_MOE_DISABLE(&htim8);
        for(;;);
    }
}

// This is the callback from the ADC that we expect after the PWM has triggered an ADC conversion.
//@TODO: Document how the phasing is done
static void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc) {

    //@TODO get rid of statics when using more than one motor
    static float phB_DC_calib = 0.0f;
    static float phC_DC_calib = 0.0f;
    #define calib_tau 0.2f //@TOTO make more easily configurable
    static const float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

    //Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    float current = phase_current_from_adcval(ADCValue, 0);

    // Check if this trigger was the CC4 channel, used for actual current measurement at SVM vector 0
    // or the update trigger, which is used for DC_CAL measurement at SVM vector 7
    uint32_t trig_src = hadc->Instance->CR2 & ADC_CR2_JEXTSEL;
    if (trig_src == ADC_EXTERNALTRIGINJECCONV_T1_CC4) {
        //We are measuring current here
        //Set up next measurement to be DC_CAL measurement
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |=  ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
        HAL_GPIO_WritePin(M0_DC_CAL_GPIO_Port, M0_DC_CAL_Pin, GPIO_PIN_SET);

        // ADC2 and ADC3 record the phB and phC currents concurrently,
        // and their interrupts should arrive on the same clock cycle.
        // The HAL issues the callbacks in order, so ADC2 will always be processed before ADC3.
        // Therefore we store the value from ADC2 and push them both into the queue
        // when ADC3 is ready.
        // @TODO: don't use statics, will only work for 1 motor chanel
        static float phB_current;

        //Store and return, or fetch and continue
        float phC_current;
        if (hadc == &hadc2) {
            phB_current = current;
            return;
        } else if (hadc == &hadc3) {
            phC_current = current;
        } else {
            //hadc is something else, not expected
            safe_assert(0);
        }

        //Allocate mail queue storage
        Iph_BC_queue_item_t* mail_ptr;
        mail_ptr = (Iph_BC_queue_item_t*) osMailAlloc(M0_Iph_queue, 0);
        if (mail_ptr == NULL) {
            return;
        }

        //Write contents and send mail
        mail_ptr->current_phB = phB_current - phB_DC_calib;
        mail_ptr->current_phC = phC_current - phC_DC_calib;
        osMailPut(M0_Iph_queue, mail_ptr);

    } else if (trig_src == ADC_EXTERNALTRIGINJECCONV_T1_TRGO) {
        //We are measuring DC_CAL here
        //Set up next measurement to be current measurement
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |=  ADC_EXTERNALTRIGINJECCONV_T1_CC4;
        HAL_GPIO_WritePin(M0_DC_CAL_GPIO_Port, M0_DC_CAL_Pin, GPIO_PIN_RESET);

        if (hadc == &hadc2) {
            phB_DC_calib += (current - phB_DC_calib) * calib_filter_k;
        } else if (hadc == &hadc3) {
            phC_DC_calib += (current - phC_DC_calib) * calib_filter_k;
        } else {
            //hadc is something else, not expected
            safe_assert(0);
        }
        
    } else {
        safe_assert(0);
    }
}

void mark_timing() {
#define log_size 32
	static uint32_t timings[log_size];
	static int idx = 0;

	if(++idx == log_size)
		idx = 0;

	timings[idx] = htim1.Instance->CNT;
}

void motor_thread(void const * argument) {

	init_motor_control();

    float test_voltages[] = {0.3f, 0.7f};
    float test_currentsB[] = {0.0f, 0.0f};
    float test_currentsC[] = {0.0f, 0.0f};
    float test_sum[] = {0.0f, 0.0f};
    static const int num_test = sizeof(test_voltages)/sizeof(test_voltages[0]);

    for(;;) {
        for (int i = 0; i < num_test; ++i) {
            for (int rep = 0; rep < 10000; ++rep) {

                //Current measurements not occurring in a timely manner can be handled by the watchdog
                //@TODO Actually make watchdog
                //Hence we can use osWaitForever
                osEvent evt = osMailGet(M0_Iph_queue, osWaitForever);

                mark_timing();

                //Since we wait forever, we do not expect timeouts here.
                safe_assert(evt.status == osEventMail);

                //Fetch current out of the mail queue
                Iph_BC_queue_item_t* mail_ptr = evt.value.p;
                float M0_phB_current = mail_ptr->current_phB;
                float M0_phC_current = mail_ptr->current_phC;
                osMailFree(M0_Iph_queue, mail_ptr);


                test_currentsB[i] += M0_phB_current;
                test_currentsC[i] += M0_phC_current;
                test_sum[i] += 1.0f;


                int full_load = htim1.Instance->ARR;
                int half_load = full_load/2;
                float DC_bus_voltage = 12.0f;

                // float test_voltage = 0.5f;
                float test_modulation = test_voltages[i]/DC_bus_voltage;
                int timing = (int)(test_modulation * (float)half_load);

                //Test voltage along phase A
                htim1.Instance->CCR1 = half_load - timing;
                htim1.Instance->CCR2 = half_load + timing;
                htim1.Instance->CCR3 = half_load + timing;

                mark_timing();
            }
        }
    }
}

