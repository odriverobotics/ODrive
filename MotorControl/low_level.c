/* Includes ------------------------------------------------------------------*/

// Because of broken cmsis_os.h, we need to include arm_math first,
// otherwise chip specific defines are ommited
#include <stm32f4xx_hal.h> //Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4
#include <arm_math.h>

#include <low_level.h>

#include <stdlib.h>
#include <math.h>
#include <cmsis_os.h>

#include <main.h>
#include <adc.h>
#include <tim.h>
#include <spi.h>
#include <utils.h>

/* Private defines -----------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
float vbus_voltage = 12.0f; //Arbitrary non-zero inital value to avoid division by zero if ADC reading is late

Motor_t motors[] = {
    {   //M0
        .motor_thread = 0,
        .thread_ready = false,
        .timer_handle = &htim1,
        .next_timings = {TIM_PERIOD_CLOCKS/2, TIM_PERIOD_CLOCKS/2, TIM_PERIOD_CLOCKS/2},
        .current_meas = {0.0f, 0.0f},
        .DC_calib = {0.0f, 0.0f},
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
    },
    {   //M1
        .motor_thread = 0,
        .thread_ready = false,
        .timer_handle = &htim8,
        .next_timings = {TIM_PERIOD_CLOCKS/2, TIM_PERIOD_CLOCKS/2, TIM_PERIOD_CLOCKS/2},
        .current_meas = {0.0f, 0.0f},
        .DC_calib = {0.0f, 0.0f},
        .gate_driver = {
            .spiHandle = &hspi3,
            //Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M1_nCS_GPIO_Port,
            .nCSgpioNumber = M1_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false
        },
        .shunt_conductance = 1.0f/0.0005f, //[S]
        .maxcurrent = 75.0f //[A] //Note: consistent with 40v/v gain
    }
};
const int num_motors = sizeof(motors)/sizeof(motors[0]);

/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//Local view of DRV registers
//@TODO: Include these in motor object instead
static DRV_SPI_8301_Vars_t gate_driver_regs[2/*num_motors*/];

//Log to store the timing of calls to check_timing
//This is used in various places, so be sure to look for all the places it is written
#define TIMING_LOG_SIZE 32
static volatile uint16_t timing_logs[2/*num_motors*/][TIMING_LOG_SIZE];
static volatile int timing_log_index[2/*num_motors*/] = {0, 0};

/* Private function prototypes -----------------------------------------------*/
static void DRV8301_setup(Motor_t* motor, DRV_SPI_8301_Vars_t* local_regs);
static void start_adc_pwm();
static void start_pwm(TIM_HandleTypeDef* htim);
static void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
        uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset);
static float phase_current_from_adcval(uint32_t ADCValue, int motornum);
static uint16_t check_timing(TIM_HandleTypeDef* htim, volatile uint16_t* log, volatile int* idx);
static void queue_modulation(Motor_t* motor, float mod_alpha, float mod_beta);
static void wait_for_current_meas(Motor_t* motor, float* phB_current, float* phC_current);
static float measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage);
static float measure_phase_inductance(Motor_t* motor, float voltage_low, float voltage_high);

/* Function implementations --------------------------------------------------*/

// Initalises the low level motor control and then starts the motor control threads
void init_motor_control() {
    //Init gate drivers
    DRV8301_setup(&motors[0], &gate_driver_regs[0]);
    DRV8301_setup(&motors[1], &gate_driver_regs[1]);

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    // Start Encoders
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    //Wait for current sense calibration to converge
    //@TODO make timing a function of calibration filter tau
    osDelay(1500);
}

//@TODO make available from anywhere
void safe_assert(int arg) {
    if(!arg) {
        htim1.Instance->BDTR &= ~(TIM_BDTR_MOE);
        htim8.Instance->BDTR &= ~(TIM_BDTR_MOE);
        for(;;);
    }
}

// Set up the gate drivers
//@TODO stick DRV_SPI_8301_Vars_t in motor
static void DRV8301_setup(Motor_t* motor, DRV_SPI_8301_Vars_t* local_regs) {
    for (int i = 0; i < num_motors; ++i) {
        DRV8301_enable(&motor->gate_driver);
        DRV8301_setupSpi(&motor->gate_driver, local_regs);

        //@TODO we can use reporting only if we actually wire up the nOCTW pin
        local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
        //Overcurrent set to approximately 150A at 100degC. This may need tweaking.
        local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
        //20V/V on 500uOhm gives a range of +/- 150A
        //40V/V on 500uOhm gives a range of +/- 75A
        local_regs->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_40VpV;

        local_regs->SndCmd = true;
        DRV8301_writeData(&motor->gate_driver, local_regs);
        local_regs->RcvCmd = true;
        DRV8301_readData(&motor->gate_driver, local_regs);
    }
}

static void start_adc_pwm(){
    //Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    //Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_EOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_EOC);

    //Ensure that debug halting of the core doesn't leave the motor PWM running
    __HAL_DBGMCU_FREEZE_TIM1();
    __HAL_DBGMCU_FREEZE_TIM8();

    //Turn off the regular conversion trigger for the inital phase
    hadc2.Instance->CR2 &= ~ADC_CR2_EXTEN;
    hadc3.Instance->CR2 &= ~ADC_CR2_EXTEN;

    start_pwm(&htim1);
    start_pwm(&htim8);
    sync_timers(&htim1, &htim8, TIM_CLOCKSOURCE_ITR0, TIM_PERIOD_CLOCKS/2);
}

static void start_pwm(TIM_HandleTypeDef* htim){
    //Init PWM
    int half_load = TIM_PERIOD_CLOCKS/2;
    htim->Instance->CCR1 = half_load;
    htim->Instance->CCR2 = half_load;
    htim->Instance->CCR3 = half_load;

    //This hardware obfustication layer really is getting on my nerves
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);

    htim->Instance->CCR4 = 1;
    HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_4);
}

static void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
        uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset) {

    //Store intial timer configs
    uint16_t MOE_store_a = htim_a->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t MOE_store_b = htim_b->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t CR2_store = htim_a->Instance->CR2;
    uint16_t SMCR_store = htim_b->Instance->SMCR;
    //Turn off output
    htim_a->Instance->BDTR &= ~(TIM_BDTR_MOE);
    htim_b->Instance->BDTR &= ~(TIM_BDTR_MOE);
    // Disable both timer counters
    htim_a->Instance->CR1 &= ~TIM_CR1_CEN;
    htim_b->Instance->CR1 &= ~TIM_CR1_CEN;
    // Set first timer to send TRGO on counter enable
    htim_a->Instance->CR2 &= ~TIM_CR2_MMS;
    htim_a->Instance->CR2 |= TIM_TRGO_ENABLE;
    // Set Trigger Source of second timer to the TRGO of the first timer
    htim_b->Instance->SMCR &= ~TIM_SMCR_TS;
    htim_b->Instance->SMCR |= TIM_CLOCKSOURCE_ITRx;
    // Set 2nd timer to start on trigger
    htim_b->Instance->SMCR &= ~TIM_SMCR_SMS;
    htim_b->Instance->SMCR |= TIM_SLAVEMODE_TRIGGER;
    // Dir bit is read only in center aligned mode, so we clear the mode for now
    uint16_t CMS_store_a = htim_a->Instance->CR1 & TIM_CR1_CMS;
    uint16_t CMS_store_b = htim_b->Instance->CR1 & TIM_CR1_CMS;
    htim_a->Instance->CR1 &= ~TIM_CR1_CMS;
    htim_b->Instance->CR1 &= ~TIM_CR1_CMS;
    // Set both timers to up-counting state
    htim_a->Instance->CR1 &= ~TIM_CR1_DIR;
    htim_b->Instance->CR1 &= ~TIM_CR1_DIR;
    // Restore center aligned mode
    htim_a->Instance->CR1 |= CMS_store_a;
    htim_b->Instance->CR1 |= CMS_store_b;
    // set counter offset
    htim_a->Instance->CNT = count_offset;
    htim_b->Instance->CNT = 0;
    // Start Timer a
    htim_a->Instance->CR1 |= (TIM_CR1_CEN);
    // Restore timer configs
    htim_a->Instance->CR2 = CR2_store;
    htim_b->Instance->SMCR = SMCR_store;
    //restore output
    htim_a->Instance->BDTR |= MOE_store_a;
    htim_b->Instance->BDTR |= MOE_store_b;
}

static float phase_current_from_adcval(uint32_t ADCValue, int motornum) {
    float rev_gain;
    //@TODO we can shave off some clock cycles by writing a static rev_gain in the motor struct
    //when we set the gains
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

void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc) {
    static const float voltage_scale = 3.3 * 11.0f / (float)(1<<12);
    //Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    vbus_voltage = ADCValue * voltage_scale;
}

// This is the callback from the ADC that we expect after the PWM has triggered an ADC conversion.
//@TODO: Document how the phasing is done, link to timing diagram
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc) {
    #define calib_tau 0.2f //@TOTO make more easily configurable
    static const float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

    //Ensure ADCs are expected ones to simplify the logic below
    safe_assert(hadc == &hadc2 || hadc == &hadc3);

    bool current_meas_not_DC_CAL;
    Motor_t* motor;

    // Check if this trigger was the CC4 channel, used for actual current measurement at SVM vector 0
    // or the update trigger, which is used for DC_CAL measurement at SVM vector 7
    // M1 DC_CAL is a special case since due to hardware limitations, it uses the "regular" conversions
    // rather than the injected ones.
    uint32_t inj_src = hadc->Instance->CR2 & ADC_CR2_JEXTSEL;
    uint32_t reg_edge = hadc->Instance->CR2 & ADC_CR2_EXTEN;
    if (reg_edge != ADC_EXTERNALTRIGCONVEDGE_NONE) {
        //We are measuring M1 DC_CAL here
        current_meas_not_DC_CAL = false;
        motor = &motors[1];
        //Next measurement on this motor will be M1 current measurement
        HAL_GPIO_WritePin(M1_DC_CAL_GPIO_Port, M1_DC_CAL_Pin, GPIO_PIN_RESET);
        //Next measurement on this ADC will be M0 current
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN | ADC_CR2_EXTEN | ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |= (ADC_EXTERNALTRIGINJECCONVEDGE_RISING | ADC_EXTERNALTRIGINJECCONV_T1_CC4);
        //Set ADC channels for next measurement
        hadc->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, 1, 1);
        hadc->Instance->JSQR |= ADC_JSQR((hadc == &hadc2) ? ADC_CHANNEL_10 : ADC_CHANNEL_11, 1, 1);
        //Load next timings for M0 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[0].timer_handle->Instance->CCR1 = motors[0].next_timings[0];
            motors[0].timer_handle->Instance->CCR2 = motors[0].next_timings[1];
            motors[0].timer_handle->Instance->CCR3 = motors[0].next_timings[2];
        }
        //Check the timing of the sequencing
        check_timing(motor->timer_handle, timing_logs[1], &timing_log_index[1]);

    } else if (inj_src == ADC_EXTERNALTRIGINJECCONV_T1_CC4) {
        //We are measuring M0 current here
        current_meas_not_DC_CAL = true;
        motor = &motors[0];
        //Next measurement on this motor will be M0 DC_CAL measurement
        HAL_GPIO_WritePin(M0_DC_CAL_GPIO_Port, M0_DC_CAL_Pin, GPIO_PIN_SET);
        //Next measurement on this ADC will be M1 current
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN | ADC_CR2_EXTEN | ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |= (ADC_EXTERNALTRIGINJECCONVEDGE_RISING | ADC_EXTERNALTRIGINJECCONV_T8_CC4);
        //Set ADC channels for next measurement
        hadc->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, 1, 1);
        hadc->Instance->JSQR |= ADC_JSQR((hadc == &hadc2) ? ADC_CHANNEL_13 : ADC_CHANNEL_12, 1, 1);
        //Load next timings for M1 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[1].timer_handle->Instance->CCR1 = motors[1].next_timings[0];
            motors[1].timer_handle->Instance->CCR2 = motors[1].next_timings[1];
            motors[1].timer_handle->Instance->CCR3 = motors[1].next_timings[2];
        }
        //Check the timing of the sequencing
        check_timing(motor->timer_handle, timing_logs[0], &timing_log_index[0]);

    } else if (inj_src == ADC_EXTERNALTRIGINJECCONV_T8_CC4) {
        //We are measuring M1 current here
        current_meas_not_DC_CAL = true;
        motor = &motors[1];
        //Next measurement on this motor will be M1 DC_CAL measurement
        HAL_GPIO_WritePin(M1_DC_CAL_GPIO_Port, M1_DC_CAL_Pin, GPIO_PIN_SET);
        //Next measurement on this ADC will be M0 DC_CAL
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN | ADC_CR2_EXTEN | ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |= (ADC_EXTERNALTRIGINJECCONVEDGE_RISING | ADC_EXTERNALTRIGINJECCONV_T1_TRGO);
        //Set ADC channels for next measurement
        hadc->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, 1, 1);
        hadc->Instance->JSQR |= ADC_JSQR((hadc == &hadc2) ? ADC_CHANNEL_10 : ADC_CHANNEL_11, 1, 1);
        //Check the timing of the sequencing
        check_timing(motor->timer_handle, timing_logs[1], &timing_log_index[1]);

    } else if (inj_src == ADC_EXTERNALTRIGINJECCONV_T1_TRGO) {
        //We are measuring M0 DC_CAL here
        current_meas_not_DC_CAL = false;
        motor = &motors[0];
        //Next measurement on this motor will be M0 current measurement
        HAL_GPIO_WritePin(M0_DC_CAL_GPIO_Port, M0_DC_CAL_Pin, GPIO_PIN_RESET);
        //Next measurement on this ADC will be M1 DC_CAL
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN | ADC_CR2_EXTEN | ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |= ADC_EXTERNALTRIGCONVEDGE_RISING;
        //Set ADC channels for next measurement
        hadc->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, 1, 1);
        hadc->Instance->JSQR |= ADC_JSQR((hadc == &hadc2) ? ADC_CHANNEL_13 : ADC_CHANNEL_12, 1, 1);
        //Check the timing of the sequencing
        check_timing(motor->timer_handle, timing_logs[0], &timing_log_index[0]);

    } else {
        safe_assert(0);
    }

    uint32_t ADCValue;
    if (reg_edge != ADC_EXTERNALTRIGCONVEDGE_NONE) {
        ADCValue = HAL_ADC_GetValue(hadc);
    } else {
        ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    }
    //@TODO remove hardcoded motornum
    float current = phase_current_from_adcval(ADCValue, 0);

    if (current_meas_not_DC_CAL) {
        // ADC2 and ADC3 record the phB and phC currents concurrently,
        // and their interrupts should arrive on the same clock cycle.
        // We dispatch the callbacks in order, so ADC2 will always be processed before ADC3.
        // Therefore we store the value from ADC2 and signal the thread that the
        // measurement is ready when we recieve the ADC3 measurement

        //return or continue
        if (hadc == &hadc2) {
            motor->current_meas.phB = current - motor->DC_calib.phB;
            return;
        } else {
            motor->current_meas.phC = current - motor->DC_calib.phC;
        }
        // Trigger motor thread
        if (motor->thread_ready)
            osSignalSet(motor->motor_thread, M_SIGNAL_PH_CURRENT_MEAS);

    } else {
        // DC_CAL measurement
        if (hadc == &hadc2) {
            motor->DC_calib.phB += (current - motor->DC_calib.phB) * calib_filter_k;
        } else {
            motor->DC_calib.phC += (current - motor->DC_calib.phC) * calib_filter_k;
        }
    }
}

static uint16_t check_timing(TIM_HandleTypeDef* htim, volatile uint16_t* log, volatile int* idx) {
    uint16_t timing = htim->Instance->CNT;
    bool down = htim->Instance->CR1 & TIM_CR1_DIR;
    if (down) {
        uint16_t delta = TIM_PERIOD_CLOCKS - timing;
        timing = TIM_PERIOD_CLOCKS + delta;
    }

    if (log != NULL && idx != NULL) {
        if(++(*idx) == TIMING_LOG_SIZE)
            *idx = 0;
        log[*idx] = timing;
    }

    return timing;
}

static void wait_for_current_meas(Motor_t* motor, float* phB_current, float* phC_current) {
    //Current measurements not occurring in a timely manner can be handled by the watchdog
    //@TODO Actually make watchdog
    //Hence we can use osWaitForever
    osEvent evt = osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);

    //Since we wait forever, we do not expect timeouts here.
    safe_assert(evt.status == osEventSignal);

    //Fetch currents
    *phB_current = motor->current_meas.phB;
    *phC_current = motor->current_meas.phC;
}

static float measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage) {
    static const float kI = 10.0f; //[(V/s)/A]
    static const int num_test_cycles = 3.0f / CURRENT_MEAS_PERIOD;

    float test_voltage = 0.0f;
    for (int i = 0; i < num_test_cycles; ++i) {
        float IphB, IphC;
        wait_for_current_meas(motor, &IphB, &IphC);
        float Ialpha = -0.5f * (IphB + IphC);
        test_voltage += (kI * CURRENT_MEAS_PERIOD) * (test_current - Ialpha);
        if (test_voltage > max_voltage) test_voltage = max_voltage;
        if (test_voltage < -max_voltage) test_voltage = -max_voltage;

        //Test voltage along phase A
        float mod = test_voltage / ((2.0f / 3.0f) * vbus_voltage);
        queue_modulation(motor, mod, 0.0f);

        //Check we meet deadlines after queueing
        safe_assert(check_timing(motor->timer_handle, NULL, NULL) < TIM_PERIOD_CLOCKS);
    }

    //De-energize motor
    queue_modulation(motor, 0.0f, 0.0f);

    float phase_resistance = test_voltage / test_current;
    return phase_resistance;
}

static void queue_modulation(Motor_t* motor, float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    motor->next_timings[0] = (uint16_t)(tA * (float)TIM_PERIOD_CLOCKS);
    motor->next_timings[1] = (uint16_t)(tB * (float)TIM_PERIOD_CLOCKS);
    motor->next_timings[2] = (uint16_t)(tC * (float)TIM_PERIOD_CLOCKS);
}

static float measure_phase_inductance(Motor_t* motor, float voltage_low, float voltage_high) {
    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    static const int num_cycles = 5000;
    for (int t = 0; t < num_cycles; ++t) {
        for (int i = 0; i < 2; ++i) {

            float phB_current, phC_current;
            wait_for_current_meas(motor, &phB_current, &phC_current);
            Ialphas[i] += -phB_current - phC_current;

            //Test voltage along phase A
            float mod = test_voltages[i] / ((2.0f / 3.0f) * vbus_voltage);
            queue_modulation(motor, mod, 0.0f);

            //Check we meet deadlines after queueing
            safe_assert(check_timing(motor->timer_handle, NULL, NULL) < TIM_PERIOD_CLOCKS);
        }
    }

    float v_L = 0.5f * (voltage_high - voltage_low);
    //Note: A more correct formula would also take into account that there is a finite timestep.
    //However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (CURRENT_MEAS_PERIOD * (float)num_cycles);
    float L = v_L / dI_by_dt;
    return L;
}

static void scan_motor(Motor_t* motor, float omega, float voltage_magnitude) {
    for(;;) {
        for (float ph = 0.0f; ph < 2.0f * M_PI; ph += omega * CURRENT_MEAS_PERIOD) {
            float IphB, IphC;
            wait_for_current_meas(motor, &IphB, &IphC);

            float c = arm_cos_f32(ph);
            float s = arm_sin_f32(ph);
            float mod_alpha = (c * voltage_magnitude) / ((2.0f / 3.0f) * vbus_voltage);
            float mod_beta = (s * voltage_magnitude) / ((2.0f / 3.0f) * vbus_voltage);
            queue_modulation(motor, mod_alpha, mod_beta);

            //Check we meet deadlines after queueing
            safe_assert(check_timing(motor->timer_handle, NULL, NULL) < TIM_PERIOD_CLOCKS);

            int16_t h3cnt = htim3.Instance->CNT;
            int16_t h4cnt = htim4.Instance->CNT;
            if (abs(h3cnt) > 1000 || abs(h4cnt) > 1000){
                int test = 1;
            }
        }
    }
}

void motor_thread(void const * argument) {
    Motor_t* motor = (Motor_t*)argument;
    motor->motor_thread = osThreadGetId();
    motor->thread_ready = true;

    float test_current = 4.0f;
    float R = measure_phase_resistance(motor, test_current, 1.0f);
    float L = measure_phase_inductance(motor, -1.0f, 1.0f);
    if (motor == &motors[0]) {
        scan_motor(motor, 50.0f, test_current * R);
    } else {
        scan_motor(motor, 10.0f, test_current * R);
    }

    //De-energize motor
    queue_modulation(motor, 0.0f, 0.0f);
}

