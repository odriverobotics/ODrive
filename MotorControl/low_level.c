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

//@TODO: Migrate to C++, clearly we are actually doing object oriented code here...
Motor_t motors[] = {
    {   //M0
        .motor_thread = 0,
        .thread_ready = false,
        .motor_timer = &htim1,
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
        .current_control = {
            .current_lim = 75.0f, //[A] //Note: consistent with 40v/v gain
            .p_gain = 0.0f, // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f, // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f
        },
        .rotor = {
            .encoder_timer = &htim3,
            .encoder_offset = 0,
            .encoder_state = 0,
            .phase = 0.0f, // [rad]
            .pll_pos = 0.0f, // [rad]
            .pll_vel = 0.0f, // [rad/s]
            .pll_kp = 0.0f, // [rad/s / rad]
            .pll_ki = 0.0f // [(rad/s^2) / rad]
        }
    },
    {   //M1
        .motor_thread = 0,
        .thread_ready = false,
        .motor_timer = &htim8,
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
        .current_control = {
            .current_lim = 75.0f, //[A] //Note: consistent with 40v/v gain
            .p_gain = 0.0f, // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f, // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f
        },
        .rotor = {
            .encoder_timer = &htim4,
            .encoder_offset = 0,
            .encoder_state = 0,
            .phase = 0.0f,
            .pll_pos = 0.0f, // [rad]
            .pll_vel = 0.0f, // [rad/s]
            .pll_kp = 0.0f, // [rad/s / rad]
            .pll_ki = 0.0f // [(rad/s^2) / rad]
        }
    }
};
const int num_motors = sizeof(motors)/sizeof(motors[0]);

/* Private constant data -----------------------------------------------------*/
static const float one_by_sqrt3 = 0.57735026919f;
static const float sqrt3_by_2 = 0.86602540378;

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
static void queue_voltage_timings(Motor_t* motor, float v_alpha, float v_beta);
static float measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage);
static float measure_phase_inductance(Motor_t* motor, float voltage_low, float voltage_high);
static int16_t calib_enc_offset(Motor_t* motor, float voltage_magnitude);

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
            motors[0].motor_timer->Instance->CCR1 = motors[0].next_timings[0];
            motors[0].motor_timer->Instance->CCR2 = motors[0].next_timings[1];
            motors[0].motor_timer->Instance->CCR3 = motors[0].next_timings[2];
        }
        //Check the timing of the sequencing
        check_timing(motor->motor_timer, timing_logs[1], &timing_log_index[1]);

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
            motors[1].motor_timer->Instance->CCR1 = motors[1].next_timings[0];
            motors[1].motor_timer->Instance->CCR2 = motors[1].next_timings[1];
            motors[1].motor_timer->Instance->CCR3 = motors[1].next_timings[2];
        }
        //Check the timing of the sequencing
        check_timing(motor->motor_timer, timing_logs[0], &timing_log_index[0]);

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
        check_timing(motor->motor_timer, timing_logs[1], &timing_log_index[1]);

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
        check_timing(motor->motor_timer, timing_logs[0], &timing_log_index[0]);

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

static float measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage) {
    static const float kI = 10.0f; //[(V/s)/A]
    static const int num_test_cycles = 3.0f / CURRENT_MEAS_PERIOD;

    float test_voltage = 0.0f;
    for (int i = 0; i < num_test_cycles; ++i) {
        osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
        float Ialpha = -0.5f * (motor->current_meas.phB + motor->current_meas.phC);
        test_voltage += (kI * CURRENT_MEAS_PERIOD) * (test_current - Ialpha);
        if (test_voltage > max_voltage) test_voltage = max_voltage;
        if (test_voltage < -max_voltage) test_voltage = -max_voltage;

        //Test voltage along phase A
        queue_voltage_timings(motor, test_voltage, 0.0f);

        //Check we meet deadlines after queueing
        safe_assert(check_timing(motor->motor_timer, NULL, NULL) < TIM_PERIOD_CLOCKS);
    }

    //De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);

    float phase_resistance = test_voltage / test_current;
    return phase_resistance;
}

static void queue_modulation_timings(Motor_t* motor, float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    motor->next_timings[0] = (uint16_t)(tA * (float)TIM_PERIOD_CLOCKS);
    motor->next_timings[1] = (uint16_t)(tB * (float)TIM_PERIOD_CLOCKS);
    motor->next_timings[2] = (uint16_t)(tC * (float)TIM_PERIOD_CLOCKS);
}

static void queue_voltage_timings(Motor_t* motor, float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    queue_modulation_timings(motor, mod_alpha, mod_beta);
}

static float measure_phase_inductance(Motor_t* motor, float voltage_low, float voltage_high) {
    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    static const int num_cycles = 5000;
    for (int t = 0; t < num_cycles; ++t) {
        for (int i = 0; i < 2; ++i) {
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
            Ialphas[i] += -motor->current_meas.phB - motor->current_meas.phC;

            //Test voltage along phase A
            queue_voltage_timings(motor, test_voltages[i], 0.0f);

            //Check we meet deadlines after queueing
            safe_assert(check_timing(motor->motor_timer, NULL, NULL) < TIM_PERIOD_CLOCKS);
        }
    }

    float v_L = 0.5f * (voltage_high - voltage_low);
    //Note: A more correct formula would also take into account that there is a finite timestep.
    //However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (CURRENT_MEAS_PERIOD * (float)num_cycles);
    float L = v_L / dI_by_dt;
    return L;
}

//TODO: Do the scan with current, not voltage!
//TODO: add check_timing
static int16_t calib_enc_offset(Motor_t* motor, float voltage_magnitude) {
    static const float start_lock_duration = 1.0f;
    static const int num_steps = 1024;
    static const float dt_step = 1.0f/500.0f;
    static const float scan_range = 4.0f * M_PI;
    const float step_size = scan_range / (float)num_steps; //TODO handle const expressions better (maybe switch to C++ ?)

    int32_t encvaluesum = 0;

    //go to rotor zero phase for 2s to get ready to scan
    for (int i = 0; i < start_lock_duration*CURRENT_MEAS_HZ; ++i) {
        osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
        queue_voltage_timings(motor, voltage_magnitude, 0.0f);
    }
    //scan forwards
    for (float ph = -scan_range / 2.0f; ph < scan_range / 2.0f; ph += step_size) {
        for (int i = 0; i < dt_step*(float)CURRENT_MEAS_HZ; ++i) {
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta  = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);
        }
        //TODO actual unit conversion
        encvaluesum += (int16_t)motor->rotor.encoder_timer->Instance->CNT;
    }

    //check direction
    //TODO ability to handle both encoder directions
    safe_assert((int16_t)motor->rotor.encoder_timer->Instance->CNT > 0);

    //scan backwards
    for (float ph = scan_range / 2.0f; ph > -scan_range / 2.0f; ph -= step_size) {
        for (int i = 0; i < dt_step*(float)CURRENT_MEAS_HZ; ++i) {
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta  = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);
        }
        //TODO actual unit conversion
        encvaluesum += (int16_t)motor->rotor.encoder_timer->Instance->CNT;
    }

    int16_t offset = encvaluesum / (num_steps * 2);
    return offset;
}

static void scan_motor_loop(Motor_t* motor, float omega, float voltage_magnitude) {
    for(;;) {
        for (float ph = 0.0f; ph < 2.0f * M_PI; ph += omega * CURRENT_MEAS_PERIOD) {
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta  = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);

            //Check we meet deadlines after queueing
            safe_assert(check_timing(motor->motor_timer, NULL, NULL) < TIM_PERIOD_CLOCKS);
        }
    }
}

static void update_rotor(Rotor_t* rotor) {
    //@TODO stick parameter into struct
    #define QCPR (600*4)
    static const float elec_rad_per_enc = 7.0 * 2 * M_PI * (1.0f / (float)QCPR);

    //update internal encoder state
    int16_t delta_enc = (int16_t)rotor->encoder_timer->Instance->CNT - (int16_t)rotor->encoder_state;
    rotor->encoder_state += (int32_t)delta_enc;

    //compute electrical phase
    float ph = elec_rad_per_enc * ((rotor->encoder_state % QCPR) - rotor->encoder_offset);
    ph = fmodf(ph, 2*M_PI);
    rotor->phase = ph;

    //run pll (for now pll is in units of encoder counts)
    //@TODO pll_pos runs out of precision very quickly here! Perhaps decompose into integer and fractional part?
    // Predict current pos
    rotor->pll_pos += CURRENT_MEAS_PERIOD * rotor->pll_vel;
    // discrete phase detector
    float delta_pos = (float)(rotor->encoder_state - (int32_t)floorf(rotor->pll_pos));
    // pll feedback
    rotor->pll_pos += CURRENT_MEAS_PERIOD * rotor->pll_kp * delta_pos;
    rotor->pll_vel += CURRENT_MEAS_PERIOD * rotor->pll_ki * delta_pos;
}

static void FOC_voltage_loop(Motor_t* motor, float v_d, float v_q) {
    for (;;) {
        osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
        update_rotor(&motor->rotor);

        float c = arm_cos_f32(motor->rotor.phase);
        float s = arm_sin_f32(motor->rotor.phase);
        float v_alpha = c*v_d - s*v_q;
        float v_beta  = c*v_q + s*v_d;
        queue_voltage_timings(motor, v_alpha, v_beta);

        //Check we meet deadlines after queueing
        safe_assert(check_timing(motor->motor_timer, NULL, NULL) < TIM_PERIOD_CLOCKS);
    }
}

static void FOC_current(Motor_t* motor, float Id_des, float Iq_des) {
    Current_control_t* ictrl = &motor->current_control;

    //Clarke transform
    float Ialpha = -motor->current_meas.phB - motor->current_meas.phC;
    float Ibeta = one_by_sqrt3 * (motor->current_meas.phB - motor->current_meas.phC);

    //Park transform
    float c = arm_cos_f32(motor->rotor.phase);
    float s = arm_sin_f32(motor->rotor.phase);
    float Id = c*Ialpha + s*Ibeta;
    float Iq = c*Ibeta  - s*Ialpha;

    //Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    //@TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    //@TODO current limit
    //Apply PI control
    float Vd = ictrl->v_current_control_integral_d + Ierr_d * ictrl->p_gain;
    float Vq = ictrl->v_current_control_integral_q + Ierr_q * ictrl->p_gain;

    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_d = vfactor * Vd;
    float mod_q = vfactor * Vq;

    //Vector modulation saturation, lock integrator if saturated
    //@TODO make maximum modulation configurable (currently 90%)
    float mod_scalefactor = 0.90f * sqrt3_by_2 * 1.0f/sqrtf(mod_d*mod_d + mod_q*mod_q);
    if (mod_scalefactor < 1.0f)
    {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
    } else {
        //@TODO look into fancier anti integrator windup than simple locking
        ictrl->v_current_control_integral_d += Ierr_d * (ictrl->i_gain * CURRENT_MEAS_PERIOD);
        ictrl->v_current_control_integral_q += Ierr_q * (ictrl->i_gain * CURRENT_MEAS_PERIOD);
    }

    // Compute estimated bus current
    // *IbusEst = mod_d * Id + mod_q * Iq;

    // Inverse park transform
    float mod_alpha = c*mod_d - s*mod_q;
    float mod_beta  = c*mod_q + s*mod_d;

    // Apply SVM
    queue_modulation_timings(motor, mod_alpha, mod_beta);

    //Check we meet deadlines after queueing
    //@TODO: For M1 the deadline is actually 1.5 * TIM_PERIOD_CLOCKS, double check and implement
    safe_assert(check_timing(motor->motor_timer, NULL, NULL) < TIM_PERIOD_CLOCKS);
}

static void control_velocity_loop(Motor_t* motor) {
    static const float k_vel = 10.0f / 10000.0f; // [A/(counts/s)]
    static const float test_vel = 10000.0f; // [counts/s]
    for (;;) {
        osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
        update_rotor(&motor->rotor);

        float v_err = test_vel - motor->rotor.pll_vel;
        float Iq = k_vel * v_err;
        float Ilim = motor->current_control.current_lim;
        if (Iq > Ilim) Iq = Ilim;
        if (Iq < -Ilim) Iq = -Ilim;
        FOC_current(motor, 0.0f, Iq);
    }
}

void motor_thread(void const * argument) {
    Motor_t* motor = (Motor_t*)argument;
    motor->motor_thread = osThreadGetId();
    motor->thread_ready = true;

    float test_current = 4.0f;
    float R = measure_phase_resistance(motor, test_current, 1.0f);
    float L = measure_phase_inductance(motor, -1.0f, 1.0f);
    motor->rotor.encoder_offset = calib_enc_offset(motor, test_current * R);

    //Only run tests on M0 for now
    if (motor == &motors[1]) {
        FOC_voltage_loop(motor, 0.0f, 0.0f);
    }

    //Calculate current control gains
    float current_control_bandwidth = 2000.0f; // [rad/s]
    motor->current_control.p_gain = current_control_bandwidth * L;
    float plant_pole = R/L;
    motor->current_control.i_gain = plant_pole * motor->current_control.p_gain;

    //Calculate rotor pll gains
    float rotor_pll_bandwidth = 2000.0f; // [rad/s]
    motor->rotor.pll_kp = 2.0f * rotor_pll_bandwidth;
    //Check that we don't get problems with discrete time approximation
    safe_assert(CURRENT_MEAS_PERIOD * motor->rotor.pll_kp < 1.0f);
    //Critically damped
    motor->rotor.pll_ki = 0.25f * (motor->rotor.pll_kp * motor->rotor.pll_kp);

    // scan_motor(motor, 50.0f, test_current * R);
    // FOC_voltage_loop(motor, 0.0f, 0.8f);
    // FOC_current(motor, 0.0f, 0.0f);
    control_velocity_loop(motor);

    //De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);
}

