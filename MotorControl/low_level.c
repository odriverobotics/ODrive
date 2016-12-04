
#include <low_level.h>

#include <cmsis_os.h>
#include <math.h>
#include <stdlib.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#include <main.h>
#include <adc.h>
#include <tim.h>
#include <spi.h>
#include <utils.h>

// Global variables
Motor_t motors[] = {
    {   //M0
        /* .motor_thread to be set by thread at thread start */
        .timer_handle = &htim1,
        .current_meas = {0.0f, 0.0f},
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
const int num_motors = sizeof(motors)/sizeof(motors[0]);

// Private variables

//Local view of DRV registers
//@TODO: Include these in motor object instead
static DRV_SPI_8301_Vars_t gate_driver_regs[1/*num_motors*/];

//@TODO HACK Do actual voltage measurement
static const float hack_dc_bus_voltage = 12.0f;

// Private function prototypes
void safe_assert(int arg);
static void DRV8301_setup();
static void init_encoders();
static void start_adc_pwm();
static float phase_current_from_adcval(uint32_t ADCValue, int motornum);
static void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc);
static bool check_timing();
static void set_timings(Motor_t* motor, float tA, float tB, float tC);
static void wait_for_current_meas(Motor_t* motor, float* phB_current, float* phC_current);
static float measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage);

//Special function name for ADC callback.
//Automatically registered if defined.
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // check_timing();
    pwm_trig_adc_cb(hadc);
}

void init_motor_control() {
    //Init gate drivers
    DRV8301_setup();

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    //Wait for current sense calibration to converge
    //@TODO make timing a function of calibration filter tau
    osDelay(500);
}

//@TODO make available from anywhere
void safe_assert(int arg) {
    if(!arg) {
        __HAL_TIM_MOE_DISABLE(&htim1);
        __HAL_TIM_MOE_DISABLE(&htim8);
        for(;;);
    }
}

// Set up the gate drivers
static void DRV8301_setup() {
    for (int i = 0; i < num_motors; ++i) {
        DRV8301_enable(&motors[i].gate_driver);
        DRV8301_setupSpi(&motors[i].gate_driver, &gate_driver_regs[i]);

        //@TODO we can use reporting only if we actually wire up the nOCTW pin
        gate_driver_regs[i].Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
        //Overcurrent set to approximately 150A at 100degC. This may need tweaking.
        gate_driver_regs[i].Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
        //20V/V on 500uOhm gives a range of +/- 150A
        //40V/V on 500uOhm gives a range of +/- 75A
        gate_driver_regs[i].Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_40VpV;

        gate_driver_regs[i].SndCmd = true;
        DRV8301_writeData(&motors[i].gate_driver, &gate_driver_regs[i]);
        gate_driver_regs[i].RcvCmd = true;
        DRV8301_readData(&motors[i].gate_driver, &gate_driver_regs[i]);
    }
}

static void start_pwm(TIM_HandleTypeDef htim){
    //Init PWM
    int half_load = htim.Instance->ARR/2;
    htim.Instance->CCR1 = half_load;
    htim.Instance->CCR2 = half_load;
    htim.Instance->CCR3 = half_load;

    //This hardware obfustication layer really is getting on my nerves
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim, TIM_CHANNEL_3);

    htim.Instance->CCR4 = 1;
    HAL_TIM_PWM_Start_IT(&htim, TIM_CHANNEL_4);

    //Turn off output
    //__HAL_TIM_MOE_DISABLE(&htim);
}

static void sync_timers(TIM_HandleTypeDef htim_a, TIM_HandleTypeDef htim_b,
		uint16_t internal_trigger_source, uint16_t count_offset) {

	//Store intial timer configs
    uint16_t MOE_store_a = htim_a.Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t MOE_store_b = htim_b.Instance->BDTR & (TIM_BDTR_MOE);

	uint16_t CR2_store = htim_a.Instance->CR2;
	uint16_t SMCR_store = htim_b.Instance->SMCR;

    //Turn off output
    __HAL_TIM_MOE_DISABLE(&htim_a);
    __HAL_TIM_MOE_DISABLE(&htim_b);

	/* Disable both timer counters*/
	htim_a.Instance->CR1 &= ~TIM_CR1_CEN;
	htim_b.Instance->CR1 &= ~TIM_CR1_CEN;

	/* Set first timer to send TRGO on counter enable*/
	htim_a.Instance->CR2 &= ~TIM_CR2_MMS;
	htim_a.Instance->CR2 |= TIM_TRGO_ENABLE;

	/* Set Trigger Source of second timer to the TRGO of the first timer*/
	htim_b.Instance->SMCR &= ~TIM_SMCR_TS;
	htim_b.Instance->SMCR |= TIM_CLOCKSOURCE_ITR0;

	/* Set 2nd timer to start on trigger*/
	htim_b.Instance->SMCR &= ~TIM_SMCR_SMS;
	htim_b.Instance->SMCR |= TIM_SLAVEMODE_TRIGGER;

	// set counter offset
	htim_a.Instance->CNT = 0;
	htim_b.Instance->CNT = count_offset;

	/* Start Timer 1*/
	htim_a.Instance->CR1 |= (TIM_CR1_CEN);

	/* Restore timer configs */
	htim_a.Instance->CR2 = CR2_store;
	htim_b.Instance->SMCR = SMCR_store;

    //restore output
    htim_a.Instance->BDTR |= MOE_store_a;
    htim_b.Instance->BDTR |= MOE_store_b;
}

static void init_encoders() {
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

    start_pwm(htim1);
    // start_pwm(htim8);
    // sync_timers(htim1, htim8, TIM_CLOCKSOURCE_ITR0, htim1.Instance->ARR/2);
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
        default:
            rev_gain = 0.0f; //to stop warning
            safe_assert(0);
    }

    int adcval_bal = (int)ADCValue - (1<<11);
    float amp_out_volt = (3.3f/(float)(1<<12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * rev_gain;
    float current = shunt_volt * motors[motornum].shunt_conductance;
    return current;
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
        //We are measuring M0 current here
        //Set up next measurement to be M0 DC_CAL measurement
        // @TODO add M1 to sequence
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |=  ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
        HAL_GPIO_WritePin(M0_DC_CAL_GPIO_Port, M0_DC_CAL_Pin, GPIO_PIN_SET);
        Motor_t* motor = &motors[0];

        // ADC2 and ADC3 record the phB and phC currents concurrently,
        // and their interrupts should arrive on the same clock cycle.
        // The HAL issues the callbacks in order, so ADC2 will always be processed before ADC3.
        // Therefore we only store the value from ADC2 and signal the thread that the
        // measurement is ready when we recieve the ADC3 measurement

        //return or continue
        if (hadc == &hadc2) {
            motor->current_meas.phB = current;
            return;
        } else if (hadc == &hadc3) {
            motor->current_meas.phC = current;
        } else {
            //hadc is something else, not expected
            safe_assert(0);
        }

        // Trigger motor thread
        osSignalSet(motor->motor_thread, M_SIGNAL_PH_CURRENT_MEAS);
        check_timing();

    } else if (trig_src == ADC_EXTERNALTRIGINJECCONV_T1_TRGO) {
        //We are measuring M0 DC_CAL here
        //Set up next measurement to be M0 current measurement
        // @TODO Add M1 to sequence
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

static bool check_timing() {
#define log_size 32
    static uint16_t timings[log_size];
    static int idx = 0;

    uint16_t timing = htim1.Instance->CNT;
    bool down = htim1.Instance->CR1 & TIM_CR1_DIR;
    if (down) {
        uint16_t arr = htim1.Instance->ARR;
        uint16_t delta = arr - timing;
        timing = arr + delta;
    }

    if(++idx == log_size)
        idx = 0;

    timings[idx] = timing;
    return down;
}


//@TODO: having this in a function may be a bit redundant, as it is so simple and only used in one place
static void wait_for_current_meas(Motor_t* motor, float* phB_current, float* phC_current) {
    //Current measurements not occurring in a timely manner can be handled by the watchdog
    //@TODO Actually make watchdog
    //Hence we can use osWaitForever
    osEvent evt = osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
    check_timing();

    //Since we wait forever, we do not expect timeouts here.
    safe_assert(evt.status == osEventSignal);

    //Fetch currents
    *phB_current = motor->current_meas.phB;
    *phC_current = motor->current_meas.phC;
}


static float measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage) {
    static const float kI = 0.2f; //[(V/s)/A]
    static float test_voltage = 0.0f;
    static const int num_test_cycles = 10.0f / CURRENT_MEAS_PERIOD;

    //@TODO: Fixed gain is dangerous for low impedance motors
    //@TODO: Fixed measurement time is dangerous for high impedance motors
    // We should do a geometric sequence of voltage instead.
    for (int i = 0; i < num_test_cycles; ++i) {
        float IphB, IphC;
        wait_for_current_meas(motor, &IphB, &IphC);
        float Ialpha = -0.5f * (IphB + IphC);
        test_voltage += (kI * CURRENT_MEAS_PERIOD) * (test_current - Ialpha);
        if (test_voltage > max_voltage) test_voltage = max_voltage;
        if (test_voltage < -max_voltage) test_voltage = -max_voltage;
        float mod = test_voltage/hack_dc_bus_voltage;

        //Test voltage along phase A
        float tA, tB, tC;
        SVM(mod, 0.0f, &tA, &tB, &tC);
        set_timings(&motors[0], tA, tB, tC);
    }

    //De-energize motor
    set_timings(&motors[0], 0.5f, 0.5f, 0.5f);;

    float phase_resistance = test_voltage / test_current;
    return phase_resistance;
}


//Set the rising edge timings (0.0 - 1.0)
static void set_timings(Motor_t* motor, float tA, float tB, float tC) {
    TIM_TypeDef* tim = motor->timer_handle->Instance;
    uint32_t full_load = tim->ARR;

    //Test voltage along phase A
    tim->CCR1 = tA * full_load;
    tim->CCR2 = tB * full_load;
    tim->CCR3 = tC * full_load;
}

static void square_wave_test(Motor_t* motor) {
#define NUM_CYCLES 32
    float test_voltages[] = {0.2f, 1.0f};
    float mean[2][NUM_CYCLES] = {{ 0.0f }};
    float var[2][NUM_CYCLES] = {{ 0.0f }};
    int cycle_num = 1;
    static const int num_test = sizeof(test_voltages)/sizeof(test_voltages[0]);

    for(;;) {
        for (int i = 0; i < num_test; ++i) {
            for (int rep = 0; rep < NUM_CYCLES; ++rep) {

                float phB_current, phC_current;
                wait_for_current_meas(motor, &phB_current, &phC_current);

                check_timing();

                float Ialpha = -phB_current - phC_current;
                float delta = Ialpha - mean[i][rep];
                mean[i][rep] += delta * (1.0f / (float)cycle_num);
                float delta_delta_sqr = (delta * delta) - var[i][rep];
                var[i][rep] += delta_delta_sqr * (1.0f / (float)cycle_num);

                float mod = test_voltages[i]/hack_dc_bus_voltage;
                float tA, tB, tC;
                //Test voltage along phase A
                SVM(mod, 0.0f, &tA, &tB, &tC);
                set_timings(motor, tA, tB, tC);

                safe_assert(!check_timing());
            }
        }
        ++cycle_num;
    }
}

static void scan_motor(Motor_t* motor, float omega, float voltage_magnitude) {
    for(;;) {
        for (float ph = 0.0f; ph < 2.0f * M_PI; ph += omega * CURRENT_MEAS_PERIOD) {
            float IphB, IphC;
            wait_for_current_meas(motor, &IphB, &IphC);

            float c = cosf(ph);
            float s = sinf(ph);
            float mod_alpha = (c * voltage_magnitude) / hack_dc_bus_voltage;
            float mod_beta = (s * voltage_magnitude) / hack_dc_bus_voltage;

            float tA, tB, tC;
            //Test voltage along phase A
            SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
            set_timings(&motors[0], tA, tB, tC);

            if (abs(htim3.Instance->CNT) > 1000 || abs(htim4.Instance->CNT) > 1000){
                int test = 1;
            }
        }
    }
}

void motor_thread(void const * argument) {
    Motor_t* motor = (Motor_t*)argument;
    motor->motor_thread = osThreadGetId();

    init_motor_control();

    float test_current = 3.0f;
    float R = measure_phase_resistance(&motors[0], test_current, 1.0f);
    // scan_motor(&motors[0], 10.0f, test_current * R);
    square_wave_test(&motors[0]);
}

