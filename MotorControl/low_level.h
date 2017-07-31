/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include "drv8301.h"

//default timeout waiting for phase measurement signals
#define PH_CURRENT_MEAS_TIMEOUT 2 // [ms]

/* Exported types ------------------------------------------------------------*/
typedef enum {
    M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
} Motor_thread_signals_t;

typedef enum {
    ERROR_NO_ERROR,
    ERROR_PHASE_RESISTANCE_TIMING,
    ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT,
    ERROR_PHASE_RESISTANCE_OUT_OF_RANGE,
    ERROR_PHASE_INDUCTANCE_TIMING,
    ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT,
    ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE,
    ERROR_ENCODER_RESPONSE,
    ERROR_ENCODER_MEASUREMENT_TIMEOUT,
    ERROR_ADC_FAILED,
    ERROR_CALIBRATION_TIMING,
    ERROR_FOC_TIMING,
    ERROR_FOC_MEASUREMENT_TIMEOUT,
    ERROR_SCAN_MOTOR_TIMING,
    ERROR_FOC_VOLTAGE_TIMING,
    ERROR_GATEDRIVER_INVALID_GAIN,
    ERROR_PWM_SRC_FAIL,
    ERROR_UNEXPECTED_STEP_SRC,
} Error_t;

// Note: these should be sorted from lowest level of control to
// highest level of control, to allow "<" style comparisons.
typedef enum {
    CTRL_MODE_VOLTAGE_CONTROL,
    CTRL_MODE_CURRENT_CONTROL,
    CTRL_MODE_VELOCITY_CONTROL,
    CTRL_MODE_POSITION_CONTROL
} Motor_control_mode_t;

typedef struct {
    float phB;
    float phC;
} Iph_BC_t;

typedef struct {
    float current_lim; // [A]
    float p_gain; // [V/A]
    float i_gain; // [V/As]
    float v_current_control_integral_d; // [V]
    float v_current_control_integral_q; // [V]
    float Ibus; // DC bus current [A]
    // Voltage applied at end of cycle:
    float final_v_alpha; // [V]
    float final_v_beta; // [V]
} Current_control_t;

typedef enum {
    ROTOR_MODE_ENCODER,
    ROTOR_MODE_SENSORLESS,
    ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS //Run on encoder, but still run estimator for testing
} Rotor_mode_t;

typedef struct {
    float phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;
    float observer_gain; // [rad/s]
    float flux_state[2]; // [Vs]
    float V_alpha_beta_memory[2]; // [V]
    float pm_flux_linkage; // [V / (rad/s)]
} Sensorless_t;

typedef struct {
    TIM_HandleTypeDef* encoder_timer;
    int encoder_offset;
    int encoder_state;
    int motor_dir; // 1/-1 for fwd/rev alignment to encoder.
    float phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;
} Encoder_t;

#define TIMING_LOG_SIZE 16
typedef struct {
    Motor_control_mode_t control_mode;
    bool enable_step_dir;
    float counts_per_step;
    int error;
    float pos_setpoint;
    float pos_gain;
    float vel_setpoint;
    float vel_gain;
    float vel_integrator_gain;
    float vel_integrator_current;
    float vel_limit;
    float current_setpoint;
    float calibration_current;
    float phase_inductance;
    float phase_resistance;
    osThreadId motor_thread;
    bool thread_ready;
    bool enable_control; // enable/disable via usb to start motor control. will be set to false again in case of errors.requires calibration_ok=true
    bool do_calibration; //  trigger motor calibration. will be reset to false after self test
    bool calibration_ok;
    TIM_HandleTypeDef* motor_timer;
    uint16_t next_timings[3];
    uint16_t control_deadline;
    uint16_t last_cpu_time;
    Iph_BC_t current_meas;
    Iph_BC_t DC_calib;
    DRV8301_Obj gate_driver;
    DRV_SPI_8301_Vars_t gate_driver_regs; //Local view of DRV registers
    float shunt_conductance;
    float phase_current_rev_gain; //Reverse gain for ADC to Amps
    Current_control_t current_control;
    Rotor_mode_t rotor_mode;
    Encoder_t encoder;
    Sensorless_t sensorless;
    int timing_log_index;
    uint16_t timing_log[TIMING_LOG_SIZE];
} Motor_t;

typedef struct{
        int type;
        int index;
} monitoring_slot;

/* Exported constants --------------------------------------------------------*/
extern float vbus_voltage;
extern Motor_t motors[];
extern const int num_motors;

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

//Note: to control without feed forward, set feed forward terms to 0.0f.
void set_pos_setpoint(Motor_t* motor, float pos_setpoint, float vel_feed_forward, float current_feed_forward);
void set_vel_setpoint(Motor_t* motor, float vel_setpoint, float current_feed_forward);
void set_current_setpoint(Motor_t* motor, float current_setpoint);

void safe_assert(int arg);
void init_motor_control();
void step_cb(uint16_t GPIO_Pin);
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected);
void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected);

//@TODO move motor thread to high level file
void motor_thread(void const * argument);

//@TODO move cmd parsing to high level file
void motor_parse_cmd(uint8_t* buffer, int len);

#endif //__LOW_LEVEL_H
