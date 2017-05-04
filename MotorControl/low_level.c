/* Includes ------------------------------------------------------------------*/

// Because of broken cmsis_os.h, we need to include arm_math first,
// otherwise chip specific defines are ommited
#include <stm32f4xx_hal.h> // Sets up the correct chip specifc defines required by arm_math
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

#define STANDALONE_MODE // Drive operates without USB communication
// #define DEBUG_PRINT

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;

// TODO stick parameter into struct
#define ENCODER_CPR (600*4)
static float elec_rad_per_enc = 7.0 * 2 * M_PI * (1.0f / (float)ENCODER_CPR);

// TODO: Migrate to C++, clearly we are actually doing object oriented code here...
// TODO: For nice encapsulation, consider not having the motor objects public
Motor_t motors[] = {
    {   // M0
        .control_mode = CTRL_MODE_CURRENT_CONTROL,
        .error = ERROR_NO_ERROR,
        .pos_setpoint = 0.0f,
        .pos_gain = 20.0f, // [(counts/s) / counts]
        .vel_setpoint = 0.0f,
        .vel_gain = 15.0f / 10000.0f, // [A/(counts/s)]
        .vel_integrator_gain = 10.0f / 10000.0f, // [A/(counts/s * s)]
        .vel_integrator_current = 0.0f, // [A]
        .vel_limit = 20000.0f, // [counts/s]
        .current_setpoint = 0.0f, // [A]
        .calibration_current = 10.0f, // [A]
        .phase_inductance = 0.0f, // to be set by measure_phase_inductance
        .phase_resistance = 0.0f, // to be set by measure_phase_resistance
        .motor_thread = 0,
        .thread_ready = false,
        .enable_control = false,
        .do_calibration = false,
        .calibration_ok = false,
        .motor_timer = &htim1,
        .next_timings = {TIM_1_8_PERIOD_CLOCKS/2, TIM_1_8_PERIOD_CLOCKS/2, TIM_1_8_PERIOD_CLOCKS/2},
        .control_deadline = TIM_1_8_PERIOD_CLOCKS,
        .current_meas = {0.0f, 0.0f},
        .DC_calib = {0.0f, 0.0f},
        .gate_driver = {
            .spiHandle = &hspi3,
            // Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M0_nCS_GPIO_Port,
            .nCSgpioNumber = M0_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false
        },
        // .gate_driver_regs Init by DRV8301_setup
        .shunt_conductance = 1.0f/0.0005f, //[S]
        .phase_current_rev_gain = 0.0f, // to be set by DRV8301_setup
        .current_control = {
            // .current_lim = 75.0f, //[A] // Note: consistent with 40v/v gain
            .current_lim = 10.0f, //[A]
            .p_gain = 0.0f, // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f, // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f,
            .Ibus = 0.0f
        },
        .rotor = {
            .encoder_timer = &htim3,
            .encoder_offset = 0,
            .encoder_state = 0,
            .motor_dir = 0, // set by calib_enc_offset
            .phase = 0.0f, // [rad]
            .pll_pos = 0.0f, // [rad]
            .pll_vel = 0.0f, // [rad/s]
            .pll_kp = 0.0f, // [rad/s / rad]
            .pll_ki = 0.0f // [(rad/s^2) / rad]
        },
        .timing_log_index = 0,
        .timing_log = {0}
    },
    {   // M1
        .control_mode = CTRL_MODE_CURRENT_CONTROL,
        .error = ERROR_NO_ERROR,
        .pos_setpoint = 0.0f,
        .pos_gain = 20.0f, // [(counts/s) / counts]
        .vel_setpoint = 0.0f,
        .vel_gain = 15.0f / 10000.0f, // [A/(counts/s)]
        .vel_integrator_gain = 10.0f / 10000.0f, // [A/(counts/s * s)]
        .vel_integrator_current = 0.0f, // [A]
        .vel_limit = 20000.0f, // [counts/s]
        .current_setpoint = 0.0f, // [A]
        .calibration_current = 10.0f, // [A]
        .phase_inductance = 0.0f, // to be set by measure_phase_inductance
        .phase_resistance = 0.0f, // to be set by measure_phase_resistance
        .motor_thread = 0,
        .thread_ready = false,
        .enable_control = false,
        .do_calibration = false,
        .calibration_ok = false,
        .motor_timer = &htim8,
        .next_timings = {TIM_1_8_PERIOD_CLOCKS/2, TIM_1_8_PERIOD_CLOCKS/2, TIM_1_8_PERIOD_CLOCKS/2},
        .control_deadline = (3*TIM_1_8_PERIOD_CLOCKS)/2,
        .current_meas = {0.0f, 0.0f},
        .DC_calib = {0.0f, 0.0f},
        .gate_driver = {
            .spiHandle = &hspi3,
            // Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M1_nCS_GPIO_Port,
            .nCSgpioNumber = M1_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false
        },
        // .gate_driver_regs Init by DRV8301_setup
        .shunt_conductance = 1.0f/0.0005f, //[S]
        .phase_current_rev_gain = 0.0f, // to be set by DRV8301_setup
        .current_control = {
            // .current_lim = 75.0f, //[A] // Note: consistent with 40v/v gain
            .current_lim = 10.0f, //[A]
            .p_gain = 0.0f, // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f, // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f,
            .Ibus = 0.0f
        },
        .rotor = {
            .encoder_timer = &htim4,
            .encoder_offset = 0,
            .encoder_state = 0,
            .motor_dir = 0, // set by calib_enc_offset
            .phase = 0.0f,
            .pll_pos = 0.0f, // [rad]
            .pll_vel = 0.0f, // [rad/s]
            .pll_kp = 0.0f, // [rad/s / rad]
            .pll_ki = 0.0f // [(rad/s^2) / rad]
        },
        .timing_log_index = 0,
        .timing_log = {0}
    }
};
const int num_motors = sizeof(motors)/sizeof(motors[0]);

/* Private constant data -----------------------------------------------------*/
static const float one_by_sqrt3 = 0.57735026919f;
static const float sqrt3_by_2 = 0.86602540378;

/* Private variables ---------------------------------------------------------*/
static float brake_resistance = 2.0f; // [ohm]

/* Monitoring */
monitoring_slot monitoring_slots[20] = {0};

/* variables exposed to usb interface via set/get/monitor
 * If you change something here, don't forget to regenerate the python interface with generate_api.py
 * ro/rw : read only/read write -> ro prevents the code generator from generating setter
 * */

float * exposed_floats [] = {
        &vbus_voltage, // ro
        &elec_rad_per_enc, // ro
        &motors[0].pos_setpoint, // rw
        &motors[0].pos_gain, // rw
        &motors[0].vel_setpoint, // rw
        &motors[0].vel_gain, // rw
        &motors[0].vel_integrator_gain, // rw
        &motors[0].vel_integrator_current, // rw
        &motors[0].vel_limit, // rw
        &motors[0].current_setpoint, // rw
        &motors[0].calibration_current, // rw
        &motors[0].phase_inductance, // ro
        &motors[0].phase_resistance, // ro
        &motors[0].current_meas.phB, // ro
        &motors[0].current_meas.phC, // ro
        &motors[0].DC_calib.phB, // rw
        &motors[0].DC_calib.phC, // rw
        &motors[0].shunt_conductance, // rw
        &motors[0].phase_current_rev_gain, // rw
        &motors[0].current_control.current_lim, // rw
        &motors[0].current_control.p_gain, // rw
        &motors[0].current_control.i_gain, // rw
        &motors[0].current_control.v_current_control_integral_d, // rw
        &motors[0].current_control.v_current_control_integral_q, // rw
        &motors[0].current_control.Ibus, // ro
        &motors[0].rotor.phase, // ro
        &motors[0].rotor.pll_pos, // rw
        &motors[0].rotor.pll_vel, // rw
        &motors[0].rotor.pll_kp, // rw
        &motors[0].rotor.pll_ki, // rw
        &motors[1].pos_setpoint, // rw
        &motors[1].pos_gain, // rw
        &motors[1].vel_setpoint, // rw
        &motors[1].vel_gain, // rw
        &motors[1].vel_integrator_gain, // rw
        &motors[1].vel_integrator_current, // rw
        &motors[1].vel_limit, // rw
        &motors[1].current_setpoint, // rw
        &motors[1].calibration_current, // rw
        &motors[1].phase_inductance, // ro
        &motors[1].phase_resistance, // ro
        &motors[1].current_meas.phB, // ro
        &motors[1].current_meas.phC, // ro
        &motors[1].DC_calib.phB, // rw
        &motors[1].DC_calib.phC, // rw
        &motors[1].shunt_conductance, // rw
        &motors[1].phase_current_rev_gain, // rw
        &motors[1].current_control.current_lim, // rw
        &motors[1].current_control.p_gain, // rw
        &motors[1].current_control.i_gain, // rw
        &motors[1].current_control.v_current_control_integral_d, // rw
        &motors[1].current_control.v_current_control_integral_q, // rw
        &motors[1].current_control.Ibus, // ro
        &motors[1].rotor.phase, // ro
        &motors[1].rotor.pll_pos, // rw
        &motors[1].rotor.pll_vel, // rw
        &motors[1].rotor.pll_kp, // rw
        &motors[1].rotor.pll_ki, // rw
};

int * exposed_ints [] = {
        (int*)&motors[0].control_mode, // rw
        &motors[0].rotor.encoder_offset, // rw
        &motors[0].rotor.encoder_state, // ro
        &motors[0].error, // rw
        (int*)&motors[1].control_mode, // rw
        &motors[1].rotor.encoder_offset, // rw
        &motors[1].rotor.encoder_state, // ro
        &motors[1].error, // rw
};

bool * exposed_bools [] = {
        &motors[0].thread_ready, // ro
        &motors[0].enable_control, // rw
        &motors[0].do_calibration, // rw
        &motors[0].calibration_ok, // ro
        &motors[1].thread_ready, // ro
        &motors[1].enable_control, // rw
        &motors[1].do_calibration, // rw
        &motors[1].calibration_ok, // ro
};


/* Private function prototypes -----------------------------------------------*/
// Command Handling
static void print_monitoring(int limit);
// Utility
static uint16_t check_timing(Motor_t* motor);
static void global_fault(int error);
static float phase_current_from_adcval(Motor_t* motor, uint32_t ADCValue);
// Initalisation
static void DRV8301_setup(Motor_t* motor);
static void start_adc_pwm();
static void start_pwm(TIM_HandleTypeDef* htim);
static void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
        uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset);
// IRQ Callbacks (are all public)
// Measurement and calibrationa
static bool measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage);
static bool measure_phase_inductance(Motor_t* motor, float voltage_low, float voltage_high);
static bool calib_enc_offset(Motor_t* motor, float voltage_magnitude);
static bool motor_calibration(Motor_t* motor);
// Test functions
static void scan_motor_loop(Motor_t* motor, float omega, float voltage_magnitude);
static void FOC_voltage_loop(Motor_t* motor, float v_d, float v_q);
// Main motor control
static void update_rotor(Rotor_t* rotor);
static void update_brake_current(float brake_current);
static void queue_modulation_timings(Motor_t* motor, float mod_alpha, float mod_beta);
static void queue_voltage_timings(Motor_t* motor, float v_alpha, float v_beta);
static bool FOC_current(Motor_t* motor, float Id_des, float Iq_des);
static void control_motor_loop(Motor_t* motor);
// Motor thread (is public)


/* Function implementations --------------------------------------------------*/

//--------------------------------
// Command Handling
// TODO move to different file
//--------------------------------

static void print_monitoring(int limit) {
    for (int i=0;i<limit;i++) {
        switch (monitoring_slots[i].type) {
        case 0:
            printf("%f\t",*exposed_floats[monitoring_slots[i].index]);
            break;
        case 1:
            printf("%u\t",*exposed_ints[monitoring_slots[i].index]);
            break;
        case 2:
            printf("%d\t",*exposed_bools[monitoring_slots[i].index]);
            break;
        default:
            i=100;
        }
    }
    printf("\n");
}

void set_pos_setpoint(Motor_t* motor, float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    motor->pos_setpoint = pos_setpoint;
    motor->vel_setpoint = vel_feed_forward;
    motor->current_setpoint = current_feed_forward;
    motor->control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    printf("POSITION_CONTROL %6.0f %3.3f %3.3f\n", motor->pos_setpoint, motor->vel_setpoint, motor->current_setpoint);
#endif
}

void set_vel_setpoint(Motor_t* motor, float vel_setpoint, float current_feed_forward) {
    motor->vel_setpoint = vel_setpoint;
    motor->current_setpoint = current_feed_forward;
    motor->control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", motor->vel_setpoint, motor->current_setpoint);
#endif
}

void set_current_setpoint(Motor_t* motor, float current_setpoint) {
    motor->current_setpoint = current_setpoint;
    motor->control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", motor->current_setpoint);
#endif
}

void motor_parse_cmd(uint8_t* buffer, int len) {

    // TODO very hacky way of terminating sscanf at end of buffer:
    // We should do some proper struct packing instead of using sscanf altogether
    buffer[len] = 0;

    // check incoming packet type
    if (buffer[0] == 'p') {
        // position control
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, current_feed_forward;
        int numscan = sscanf((const char*)buffer, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &current_feed_forward);
        if (numscan == 4 && motor_number < num_motors) {
            set_pos_setpoint(&motors[motor_number], pos_setpoint, vel_feed_forward, current_feed_forward);
        }
    } else if (buffer[0] == 'v') {
        // velocity control
        unsigned motor_number;
        float vel_feed_forward, current_feed_forward;
        int numscan = sscanf((const char*)buffer, "v %u %f %f", &motor_number, &vel_feed_forward, &current_feed_forward);
        if (numscan == 3 && motor_number < num_motors) {
            set_vel_setpoint(&motors[motor_number], vel_feed_forward, current_feed_forward);
        }
    } else if (buffer[0] == 'c') {
        // current control
        unsigned motor_number;
        float current_feed_forward;
        int numscan = sscanf((const char*)buffer, "c %u %f", &motor_number, &current_feed_forward);
        if (numscan == 2 && motor_number < num_motors) {
            set_current_setpoint(&motors[motor_number], current_feed_forward);
        }
    } else if (buffer[0] == 'g') { // GET
        // g <0:float,1:int,2:bool> index
        int type = 0;
        int index = 0;
        int numscan = sscanf((const char*)buffer, "g %u %u", &type, &index);
        if (numscan == 2) {
            switch(type){
            case 0: {
                printf("%f\n",*exposed_floats[index]);
                break;
            };
            case 1: {
                printf("%u\n",*exposed_ints[index]);
                break;
            };
            case 2: {
                printf("%d\n",*exposed_bools[index]);
                break;
            };
            }
        }
    } else if (buffer[0] == 's') { // SET
        // s <0:float,1:int,2:bool> index value
        int type = 0;
        int index = 0;
        int numscan = sscanf((const char*)buffer, "s %u %u", &type, &index);
        if (numscan == 2) {
            switch(type) {
            case 0: {
                sscanf((const char*)buffer, "s %u %u %f", &type, &index, exposed_floats[index]);
                break;
            };
            case 1: {
                sscanf((const char*)buffer, "s %u %u %u", &type, &index, exposed_ints[index]);
                break;
            };
            case 2: {
                int btmp = 0;
                sscanf((const char*)buffer, "s %u %u %d", &type, &index, &btmp);
                *exposed_bools[index] = btmp ? true : false;
                break;
            };
            }
        }
    } else if (buffer[0] == 'm') { // Setup Monitor
        // m <0:float,1:int,2:bool> index monitoring_slot
        int type = 0;
        int index = 0;
        int slot = 0;
        int numscan = sscanf((const char*)buffer, "m %u %u %u", &type, &index, &slot);
        if (numscan == 3) {
            monitoring_slots[slot].type = type;
            monitoring_slots[slot].index = index;
        }
    } else if (buffer[0] == 'o') { // Output Monitor
        int limit = 0;
        int numscan = sscanf((const char*)buffer, "o %u", &limit);
        if (numscan == 1) {
            print_monitoring(limit);
        }
    }
}


//--------------------------------
// Utility
//--------------------------------

static uint16_t check_timing(Motor_t* motor) {
    TIM_HandleTypeDef* htim = motor->motor_timer;
    uint16_t timing = htim->Instance->CNT;
    bool down = htim->Instance->CR1 & TIM_CR1_DIR;
    if (down) {
        uint16_t delta = TIM_1_8_PERIOD_CLOCKS - timing;
        timing = TIM_1_8_PERIOD_CLOCKS + delta;
    }

    if(++(motor->timing_log_index) == TIMING_LOG_SIZE){
        motor->timing_log_index = 0;
    }
    motor->timing_log[motor->timing_log_index] = timing;

    return timing;
}

static void global_fault(int error){
    // Disable motors NOW!
    for (int i = 0; i < num_motors; ++i) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motors[i].motor_timer);
    }
    // Set fault codes, etc.
    for (int i = 0; i < num_motors; ++i) {
        motors[i].error = error;
        motors[i].enable_control = false;
        motors[i].calibration_ok = false;
    }
    // disable brake resistor
    update_brake_current(0.0f);
}

static float phase_current_from_adcval(Motor_t* motor, uint32_t ADCValue) {
    int adcval_bal = (int)ADCValue - (1<<11);
    float amp_out_volt = (3.3f/(float)(1<<12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * motor->phase_current_rev_gain;
    float current = shunt_volt * motor->shunt_conductance;
    return current;
}


//--------------------------------
// Initalisation
//--------------------------------

// Initalises the low level motor control and then starts the motor control threads
void init_motor_control() {
    // Init gate drivers
    DRV8301_setup(&motors[0]);
    DRV8301_setup(&motors[1]);

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    // Start Encoders
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // Wait for current sense calibration to converge
    // TODO make timing a function of calibration filter tau
    osDelay(1500);
}

// Set up the gate drivers
static void DRV8301_setup(Motor_t* motor) {
        DRV8301_Obj* gate_driver = &motor->gate_driver;
        DRV_SPI_8301_Vars_t* local_regs = &motor->gate_driver_regs;

        DRV8301_enable(gate_driver);
        DRV8301_setupSpi(gate_driver, local_regs);

        // TODO we can use reporting only if we actually wire up the nOCTW pin
        local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
        // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
        local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
        // 20V/V on 500uOhm gives a range of +/- 150A
        // 40V/V on 500uOhm gives a range of +/- 75A
        local_regs->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_40VpV;

        switch (local_regs->Ctrl_Reg_2.GAIN) {
            case DRV8301_ShuntAmpGain_10VpV:
                motor->phase_current_rev_gain = 1.0f/10.0f;
                break;
            case DRV8301_ShuntAmpGain_20VpV:
                motor->phase_current_rev_gain = 1.0f/20.0f;
                break;
            case DRV8301_ShuntAmpGain_40VpV:
                motor->phase_current_rev_gain = 1.0f/40.0f;
                break;
            case DRV8301_ShuntAmpGain_80VpV:
                motor->phase_current_rev_gain = 1.0f/80.0f;
                break;
        }

        local_regs->SndCmd = true;
        DRV8301_writeData(gate_driver, local_regs);
        local_regs->RcvCmd = true;
        DRV8301_readData(gate_driver, local_regs);
}

static void start_adc_pwm(){
    // Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    // Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_EOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_EOC);

    // Ensure that debug halting of the core doesn't leave the motor PWM running
    __HAL_DBGMCU_FREEZE_TIM1();
    __HAL_DBGMCU_FREEZE_TIM8();

    // Turn off the regular conversion trigger for the inital phase
    hadc2.Instance->CR2 &= ~ADC_CR2_EXTEN;
    hadc3.Instance->CR2 &= ~ADC_CR2_EXTEN;

    start_pwm(&htim1);
    start_pwm(&htim8);
    // TODO: explain why this offset
    sync_timers(&htim1, &htim8, TIM_CLOCKSOURCE_ITR0, TIM_1_8_PERIOD_CLOCKS/2 - 1*128);

    // Motor output starts in the disabled state
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    // Start brake resistor PWM in floating output configuration
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS+1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

static void start_pwm(TIM_HandleTypeDef* htim){
    // Init PWM
    int half_load = TIM_1_8_PERIOD_CLOCKS/2;
    htim->Instance->CCR1 = half_load;
    htim->Instance->CCR2 = half_load;
    htim->Instance->CCR3 = half_load;

    // This hardware obfustication layer really is getting on my nerves
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

    // Store intial timer configs
    uint16_t MOE_store_a = htim_a->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t MOE_store_b = htim_b->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t CR2_store = htim_a->Instance->CR2;
    uint16_t SMCR_store = htim_b->Instance->SMCR;
    // Turn off output
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
    // restore output
    htim_a->Instance->BDTR |= MOE_store_a;
    htim_b->Instance->BDTR |= MOE_store_b;
}


//--------------------------------
// IRQ Callbacks
//--------------------------------

void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc) {
    static const float voltage_scale = 3.3 * 11.0f / (float)(1<<12);
    // Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    vbus_voltage = ADCValue * voltage_scale;
}

// This is the callback from the ADC that we expect after the PWM has triggered an ADC conversion.
// TODO: Document how the phasing is done, link to timing diagram
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc) {
    #define calib_tau 0.2f //@TOTO make more easily configurable
    static const float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

    // Ensure ADCs are expected ones to simplify the logic below
    if (!(hadc == &hadc2 || hadc == &hadc3)){
        global_fault(ERROR_ADC_FAILED);
        return;
    };

    bool current_meas_not_DC_CAL;
    Motor_t* motor;

    // Check if this trigger was the CC4 channel, used for actual current measurement at SVM vector 0
    // or the update trigger, which is used for DC_CAL measurement at SVM vector 7
    // M1 DC_CAL is a special case since due to hardware limitations, it uses the "regular" conversions
    // rather than the injected ones.
    uint32_t inj_src = hadc->Instance->CR2 & ADC_CR2_JEXTSEL;
    uint32_t reg_edge = hadc->Instance->CR2 & ADC_CR2_EXTEN;
    if (reg_edge != ADC_EXTERNALTRIGCONVEDGE_NONE) {
        // We are measuring M1 DC_CAL here
        current_meas_not_DC_CAL = false;
        motor = &motors[1];
        // Next measurement on this motor will be M1 current measurement
        HAL_GPIO_WritePin(M1_DC_CAL_GPIO_Port, M1_DC_CAL_Pin, GPIO_PIN_RESET);
        // Next measurement on this ADC will be M0 current
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN | ADC_CR2_EXTEN | ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |= (ADC_EXTERNALTRIGINJECCONVEDGE_RISING | ADC_EXTERNALTRIGINJECCONV_T1_CC4);
        // Set ADC channels for next measurement
        hadc->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, 1, 1);
        hadc->Instance->JSQR |= ADC_JSQR((hadc == &hadc2) ? ADC_CHANNEL_10 : ADC_CHANNEL_11, 1, 1);
        // Load next timings for M0 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[0].motor_timer->Instance->CCR1 = motors[0].next_timings[0];
            motors[0].motor_timer->Instance->CCR2 = motors[0].next_timings[1];
            motors[0].motor_timer->Instance->CCR3 = motors[0].next_timings[2];
        }
        // Check the timing of the sequencing
        check_timing(motor);

    } else if (inj_src == ADC_EXTERNALTRIGINJECCONV_T1_CC4) {
        // We are measuring M0 current here
        current_meas_not_DC_CAL = true;
        motor = &motors[0];
        // Next measurement on this motor will be M0 DC_CAL measurement
        HAL_GPIO_WritePin(M0_DC_CAL_GPIO_Port, M0_DC_CAL_Pin, GPIO_PIN_SET);
        // Next measurement on this ADC will be M1 current
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN | ADC_CR2_EXTEN | ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |= (ADC_EXTERNALTRIGINJECCONVEDGE_RISING | ADC_EXTERNALTRIGINJECCONV_T8_CC4);
        // Set ADC channels for next measurement
        hadc->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, 1, 1);
        hadc->Instance->JSQR |= ADC_JSQR((hadc == &hadc2) ? ADC_CHANNEL_13 : ADC_CHANNEL_12, 1, 1);
        // Load next timings for M1 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[1].motor_timer->Instance->CCR1 = motors[1].next_timings[0];
            motors[1].motor_timer->Instance->CCR2 = motors[1].next_timings[1];
            motors[1].motor_timer->Instance->CCR3 = motors[1].next_timings[2];
        }
        // Check the timing of the sequencing
        check_timing(motor);

    } else if (inj_src == ADC_EXTERNALTRIGINJECCONV_T8_CC4) {
        // We are measuring M1 current here
        current_meas_not_DC_CAL = true;
        motor = &motors[1];
        // Next measurement on this motor will be M1 DC_CAL measurement
        HAL_GPIO_WritePin(M1_DC_CAL_GPIO_Port, M1_DC_CAL_Pin, GPIO_PIN_SET);
        // Next measurement on this ADC will be M0 DC_CAL
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN | ADC_CR2_EXTEN | ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |= (ADC_EXTERNALTRIGINJECCONVEDGE_RISING | ADC_EXTERNALTRIGINJECCONV_T1_TRGO);
        // Set ADC channels for next measurement
        hadc->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, 1, 1);
        hadc->Instance->JSQR |= ADC_JSQR((hadc == &hadc2) ? ADC_CHANNEL_10 : ADC_CHANNEL_11, 1, 1);
        // Check the timing of the sequencing
        check_timing(motor);

    } else if (inj_src == ADC_EXTERNALTRIGINJECCONV_T1_TRGO) {
        // We are measuring M0 DC_CAL here
        current_meas_not_DC_CAL = false;
        motor = &motors[0];
        // Next measurement on this motor will be M0 current measurement
        HAL_GPIO_WritePin(M0_DC_CAL_GPIO_Port, M0_DC_CAL_Pin, GPIO_PIN_RESET);
        // Next measurement on this ADC will be M1 DC_CAL
        hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN | ADC_CR2_EXTEN | ADC_CR2_JEXTSEL);
        hadc->Instance->CR2 |= ADC_EXTERNALTRIGCONVEDGE_RISING;
        // Set ADC channels for next measurement
        hadc->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, 1, 1);
        hadc->Instance->JSQR |= ADC_JSQR((hadc == &hadc2) ? ADC_CHANNEL_13 : ADC_CHANNEL_12, 1, 1);
        // Check the timing of the sequencing
        check_timing(motor);

    } else {
        global_fault(ERROR_PWM_SRC_FAIL);
        return;
    }

    uint32_t ADCValue;
    if (reg_edge != ADC_EXTERNALTRIGCONVEDGE_NONE) {
        ADCValue = HAL_ADC_GetValue(hadc);
    } else {
        ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    }
    float current = phase_current_from_adcval(motor, ADCValue);

    if (current_meas_not_DC_CAL) {
        // ADC2 and ADC3 record the phB and phC currents concurrently,
        // and their interrupts should arrive on the same clock cycle.
        // We dispatch the callbacks in order, so ADC2 will always be processed before ADC3.
        // Therefore we store the value from ADC2 and signal the thread that the
        // measurement is ready when we recieve the ADC3 measurement

        // return or continue
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


//--------------------------------
// Measurement and calibration
//--------------------------------

// TODO measure all phases
static bool measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage) {
    static const float kI = 10.0f; //[(V/s)/A]
    static const int num_test_cycles = 3.0f / CURRENT_MEAS_PERIOD; // Test runs for 3s
    float test_voltage = 0.0f;
    for (int i = 0; i < num_test_cycles; ++i) {
        osEvent evt = osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT);
        if (evt.status != osEventSignal){
            motor->error = ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT;
            return false;
        }
        float Ialpha = -0.5f * (motor->current_meas.phB + motor->current_meas.phC);
        test_voltage += (kI * CURRENT_MEAS_PERIOD) * (test_current - Ialpha);
        if (test_voltage > max_voltage) test_voltage = max_voltage;
        if (test_voltage < -max_voltage) test_voltage = -max_voltage;

        // Test voltage along phase A
        queue_voltage_timings(motor, test_voltage, 0.0f);

        // Check we meet deadlines after queueing
        if (!(check_timing(motor) < motor->control_deadline)){
            motor->error = ERROR_PHASE_RESISTANCE_TIMING;
            return false;
        }
    }

    // De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);

    float R = test_voltage / test_current;
    if (R < 0.01 || R > 0.2) {
        motor->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        return false;
    }
    motor->phase_resistance = R;
    return true;
}

// TODO measure all phases
static bool measure_phase_inductance(Motor_t* motor, float voltage_low, float voltage_high) {
    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    static const int num_cycles = 5000;

    for (int t = 0; t < num_cycles; ++t) {
        for (int i = 0; i < 2; ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
                motor->error = ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT;
                return false;
            }
            Ialphas[i] += -motor->current_meas.phB - motor->current_meas.phC;

            // Test voltage along phase A
            queue_voltage_timings(motor, test_voltages[i], 0.0f);

            // Check we meet deadlines after queueing
            if (!(check_timing(motor) < motor->control_deadline)) {
                motor->error = ERROR_PHASE_INDUCTANCE_TIMING;
                return false;
            }
        }
    }

    // De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (CURRENT_MEAS_PERIOD * (float)num_cycles);
    float L = v_L / dI_by_dt;
    
    // TODO arbitrary values set for now
    if (L < 1e-6 || L > 500e-6) {
        motor->error = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        return false;
    }
    motor->phase_inductance = L;
    return true;
}

// TODO: Do the scan with current, not voltage!
// TODO: add check_timing
static bool calib_enc_offset(Motor_t* motor, float voltage_magnitude) {
    static const float start_lock_duration = 1.0f;
    static const int num_steps = 1024;
    static const float dt_step = 1.0f/500.0f;
    static const float scan_range = 4.0f * M_PI;
    const float step_size = scan_range / (float)num_steps; // TODO handle const expressions better (maybe switch to C++ ?)

    int32_t init_enc_val = (int16_t)motor->rotor.encoder_timer->Instance->CNT;
    int32_t encvaluesum = 0;

    // go to rotor zero phase for start_lock_duration to get ready to scan
    for (int i = 0; i < start_lock_duration*CURRENT_MEAS_HZ; ++i) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
            motor->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
            return false;
        }
        queue_voltage_timings(motor, voltage_magnitude, 0.0f);
    }
    // scan forwards
    for (float ph = -scan_range / 2.0f; ph < scan_range / 2.0f; ph += step_size) {
        for (int i = 0; i < dt_step*(float)CURRENT_MEAS_HZ; ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
                motor->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta  = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);
        }
        encvaluesum += (int16_t)motor->rotor.encoder_timer->Instance->CNT;
    }
    // check direction
    if ((int16_t)motor->rotor.encoder_timer->Instance->CNT > init_enc_val + 8) {
        // motor same dir as encoder
        motor->rotor.motor_dir = 1;
    } else if ((int16_t)motor->rotor.encoder_timer->Instance->CNT < init_enc_val - 8) {
        // motor opposite dir as encoder
        motor->rotor.motor_dir = -1;
    } else {
        // Encoder response error
        motor->error = ERROR_ENCODER_RESPONSE;
        return false;
    }
    // scan backwards
    for (float ph = scan_range / 2.0f; ph > -scan_range / 2.0f; ph -= step_size) {
        for (int i = 0; i < dt_step*(float)CURRENT_MEAS_HZ; ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
                motor->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta  = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);
        }
        encvaluesum += (int16_t)motor->rotor.encoder_timer->Instance->CNT;
    }

    int offset = encvaluesum / (num_steps * 2);
    motor->rotor.encoder_offset = offset;
    return true;
}

static bool motor_calibration(Motor_t* motor){
    motor->calibration_ok = false;
    motor->error = ERROR_NO_ERROR;

    // #warning(hardcoded values for SK3-5065-280kv!)
    // float R = 0.0332548246f;
    // float L = 7.97315806e-06f;

    if (!measure_phase_resistance(motor, motor->calibration_current, 1.0f))
        return false;
    if (!measure_phase_inductance(motor, -1.0f, 1.0f))
        return false;
    if (!calib_enc_offset(motor, motor->calibration_current * motor->phase_resistance))
        return false;
    
    // Calculate current control gains
    float current_control_bandwidth = 2000.0f; // [rad/s]
    motor->current_control.p_gain = current_control_bandwidth * motor->phase_inductance;
    float plant_pole = motor->phase_resistance / motor->phase_inductance;
    motor->current_control.i_gain = plant_pole * motor->current_control.p_gain;

    // Calculate rotor pll gains
    float rotor_pll_bandwidth = 1000.0f; // [rad/s]
    motor->rotor.pll_kp = 2.0f * rotor_pll_bandwidth;
    // Check that we don't get problems with discrete time approximation
    if (!(CURRENT_MEAS_PERIOD * motor->rotor.pll_kp < 1.0f)){
        motor->error = ERROR_CALIBRATION_TIMING;
        return false;
    }
    // Critically damped
    motor->rotor.pll_ki = 0.25f * (motor->rotor.pll_kp * motor->rotor.pll_kp);
    
    motor->calibration_ok = true;
    return true;
}


//--------------------------------
// Test functions
//--------------------------------

static void scan_motor_loop(Motor_t* motor, float omega, float voltage_magnitude) {
    for (;;) {
        for (float ph = 0.0f; ph < 2.0f * M_PI; ph += omega * CURRENT_MEAS_PERIOD) {
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta  = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);

            // Check we meet deadlines after queueing
            if(!(check_timing(motor) < motor->control_deadline)){
                motor->error = ERROR_SCAN_MOTOR_TIMING;
                return;
            }
        }
    }
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

        // Check we meet deadlines after queueing
        if (!(check_timing(motor) < motor->control_deadline)) {
            motor->error = ERROR_FOC_VOLTAGE_TIMING;
            return;
        }
    }
}


//--------------------------------
// Main motor control
//--------------------------------

static void update_rotor(Rotor_t* rotor) {
    // update internal encoder state
    int16_t delta_enc = (int16_t)rotor->encoder_timer->Instance->CNT - (int16_t)rotor->encoder_state;
    rotor->encoder_state += (int32_t)delta_enc;

    // compute electrical phase
    int corrected_enc = rotor->encoder_state % ENCODER_CPR;
    corrected_enc -= rotor->encoder_offset;
    corrected_enc *= rotor->motor_dir;
    float ph = elec_rad_per_enc * (float)corrected_enc;
    ph = fmodf(ph, 2*M_PI);
    rotor->phase = ph;

    // run pll (for now pll is in units of encoder counts)
    // TODO pll_pos runs out of precision very quickly here! Perhaps decompose into integer and fractional part?
    // Predict current pos
    rotor->pll_pos += CURRENT_MEAS_PERIOD * rotor->pll_vel;
    // discrete phase detector
    float delta_pos = (float)(rotor->encoder_state - (int32_t)floorf(rotor->pll_pos));
    // pll feedback
    rotor->pll_pos += CURRENT_MEAS_PERIOD * rotor->pll_kp * delta_pos;
    rotor->pll_vel += CURRENT_MEAS_PERIOD * rotor->pll_ki * delta_pos;
}

static void update_brake_current(float brake_current) {
    if (brake_current < 0.0f) brake_current = 0.0f;
    float brake_duty = brake_current * brake_resistance / vbus_voltage;

    // Duty limit at 90% to allow bootstrap caps to charge
    if (brake_duty > 0.9f) brake_duty = 0.9f;
    int high_on = TIM_APB1_PERIOD_CLOCKS * (1.0f - brake_duty);
    int low_off = high_on - TIM_APB1_DEADTIME_CLOCKS;
    if (low_off < 0) low_off = 0;

    // Safe update of low and high side timings
    // To avoid race condition, first reset timings to safe state
    // ch3 is low side, ch4 is high side
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS+1;
    htim2.Instance->CCR3 = low_off;
    htim2.Instance->CCR4 = high_on;
}

static void queue_modulation_timings(Motor_t* motor, float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    motor->next_timings[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    motor->next_timings[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    motor->next_timings[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
}

static void queue_voltage_timings(Motor_t* motor, float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    queue_modulation_timings(motor, mod_alpha, mod_beta);
}

static bool FOC_current(Motor_t* motor, float Id_des, float Iq_des) {
    Current_control_t* ictrl = &motor->current_control;

    // Clarke transform
    float Ialpha = -motor->current_meas.phB - motor->current_meas.phC;
    float Ibeta = one_by_sqrt3 * (motor->current_meas.phB - motor->current_meas.phC);

    // Park transform
    float c = arm_cos_f32(motor->rotor.phase);
    float s = arm_sin_f32(motor->rotor.phase);
    float Id = c*Ialpha + s*Ibeta;
    float Iq = c*Ibeta  - s*Ialpha;

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    // Apply PI control
    float Vd = ictrl->v_current_control_integral_d + Ierr_d * ictrl->p_gain;
    float Vq = ictrl->v_current_control_integral_q + Ierr_q * ictrl->p_gain;

    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_d = vfactor * Vd;
    float mod_q = vfactor * Vq;

    // Vector modulation saturation, lock integrator if saturated
    // TODO make maximum modulation configurable (currently 40%)
    float mod_scalefactor = 0.20f * sqrt3_by_2 * 1.0f/sqrtf(mod_d*mod_d + mod_q*mod_q);
    if (mod_scalefactor < 1.0f)
    {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // TODO make decayfactor configurable
        ictrl->v_current_control_integral_d *= 0.99f;
        ictrl->v_current_control_integral_q *= 0.99f;
    } else {
        ictrl->v_current_control_integral_d += Ierr_d * (ictrl->i_gain * CURRENT_MEAS_PERIOD);
        ictrl->v_current_control_integral_q += Ierr_q * (ictrl->i_gain * CURRENT_MEAS_PERIOD);
    }

    // Compute estimated bus current
    ictrl->Ibus = mod_d * Id + mod_q * Iq;

    // If this is last motor, update brake resistor duty
    // if (motor == &motors[num_motors-1]) {
    // Above check doesn't work if last motor is executing voltage control
    // TODO trigger this update in control_motor_loop instead,
    // and make voltage control a control mode in it.
        float Ibus_sum = 0.0f;
        for (int i = 0; i < num_motors; ++i) {
            Ibus_sum += motors[i].current_control.Ibus;
        }
        // Note: function will clip negative values to 0.0f
        update_brake_current(-Ibus_sum);
    // }

    // Inverse park transform
    float mod_alpha = c*mod_d - s*mod_q;
    float mod_beta  = c*mod_q + s*mod_d;

    // Apply SVM
    queue_modulation_timings(motor, mod_alpha, mod_beta);

    // Check we meet deadlines after queueing
    if(!(check_timing(motor) < motor->control_deadline)){
        motor->error = ERROR_FOC_TIMING;
        return false;
    }
    return true;
}

static void control_motor_loop(Motor_t* motor) {
    while (motor->enable_control) {
        if(osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal){
            motor->error = ERROR_FOC_MEASUREMENT_TIMEOUT;
            break;
        }
        update_rotor(&motor->rotor);

        // Position control
        // TODO Decide if we want to use encoder or pll position here
        float vel_des = motor->vel_setpoint;
        if (motor->control_mode >= CTRL_MODE_POSITION_CONTROL) {
            float pos_err = motor->pos_setpoint - motor->rotor.pll_pos;
            vel_des += motor->pos_gain * pos_err;
        }

        // Velocity limiting
        float vel_lim = motor->vel_limit;
        if (vel_des >  vel_lim) vel_des =  vel_lim;
        if (vel_des < -vel_lim) vel_des = -vel_lim;

        // Velocity control
        float Iq = motor->current_setpoint;
        float v_err = vel_des - motor->rotor.pll_vel;
        if (motor->control_mode >=  CTRL_MODE_VELOCITY_CONTROL) {
            Iq += motor->vel_gain * v_err;
        }

        // Velocity integral action before limiting
        Iq += motor->vel_integrator_current;

        // Apply motor direction correction
        Iq *= motor->rotor.motor_dir;

        // Current limiting
        float Ilim = motor->current_control.current_lim;
        bool limited = false;
        if (Iq > Ilim) {
            limited = true;
            Iq = Ilim;
        }
        if (Iq < -Ilim) {
            limited = true;
            Iq = -Ilim;
        }

        // Velocity integrator (behaviour dependent on limiting)
        if (motor->control_mode < CTRL_MODE_VELOCITY_CONTROL ) {
            // reset integral if not in use
            motor->vel_integrator_current = 0.0f;
        } else {
            if (limited) {
                // TODO make decayfactor configurable
                motor->vel_integrator_current *= 0.99f;
            } else {
                motor->vel_integrator_current += (motor->vel_integrator_gain * CURRENT_MEAS_PERIOD) * v_err;
            }
        }

        // Execute current command
        if(!FOC_current(motor, 0.0f, Iq)){
            break; // in case of error exit loop, motor->error has been set by FOC_current
        }
    }
}


//--------------------------------
// Motor thread
//--------------------------------

void motor_thread(void const * argument) {
    Motor_t* motor = (Motor_t*)argument;
    motor->motor_thread = osThreadGetId();
    motor->thread_ready = true;

#ifdef STANDALONE_MODE
    //Only run tests on M0 for now
    if (motor == &motors[1]) {
        // TODO: figure out why M1 MOE must be enabled to run M0 correctly
        __HAL_TIM_MOE_ENABLE(motor->motor_timer);
        FOC_voltage_loop(motor, 0.0f, 0.0f);
    }

    motor->do_calibration = true;
    motor->enable_control = true;

    set_pos_setpoint(motor, 10000.0f, 0.0f, 0.0f);
#endif

    for (;;) {
        if (motor->do_calibration) {
            __HAL_TIM_MOE_ENABLE(motor->motor_timer);// enable pwm outputs
            osDelay(10);
            motor_calibration(motor);
            if(!motor->calibration_ok){
                __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor->motor_timer);// disables pwm outputs
            }
            motor->do_calibration = false;
        }
        
        if (motor->calibration_ok && motor->enable_control) {
            __HAL_TIM_MOE_ENABLE(motor->motor_timer);
            osDelay(10);
            control_motor_loop(motor);
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor->motor_timer);
            if(motor->enable_control){ // if control is still enabled, we exited because of error
                motor->calibration_ok = false;
                motor->enable_control = false;
            }
        }
        
        queue_voltage_timings(motor, 0.0f, 0.0f);
        osDelay(100);
    }
    motor->thread_ready = false;
}
