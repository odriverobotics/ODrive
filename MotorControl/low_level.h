/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include "drv8301.h"


#define SIGNAL_TIMEOUT 100 // default timeout waiting for phase measurement signals

// so we can expose a int pointer to the motor->error variable
#define ERROR_NO_ERROR 0
#define ERROR_PHASE_RESISTANCE_TIMING 1
#define ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT 2
#define ERROR_PHASE_RESISTANCE_OUT_OF_RANGE 3
#define ERROR_PHASE_INDUCTANCE_TIMING 4
#define ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT 5
#define ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE 6
#define ERROR_ENCODER_DIRECTION 7
#define ERROR_ENCODER_MEASUREMENT_TIMEOUT 8
#define ERROR_ADC_FAILED 9
#define ERROR_SELFTEST_TIMING 10
#define ERROR_FOC_TIMING 11
#define ERROR_FOC_MEASUREMENT_TIMEOUT 12
#define ERROR_SCAN_MOTOR_TIMING 13
#define ERROR_FOC_VOLTAGE_TIMING 14
#define ERROR_GATEDRIVER_INVALID_GAIN 15
#define ERROR_PWM_SRC_FAIL 16

#define POSITION_CONTROL 0
#define VELOCITY_CONTROL 1
#define CURRENT_CONTROL 2

/* Exported types ------------------------------------------------------------*/
typedef enum {
    M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
} Motor_thread_signals_t;

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
} Current_control_t;

typedef struct {
    TIM_HandleTypeDef* encoder_timer;
    int encoder_offset;
    int encoder_state;
    float phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;
} Rotor_t;

#define TIMING_LOG_SIZE 16
typedef struct {
    int control_mode;
    int error;
    float pos_setpoint;
    float pos_gain;
    float vel_setpoint;
    float vel_gain;
    float vel_integrator_gain;
    float vel_integrator_current;
    float vel_limit;
    float current_setpoint;
    float selftest_current;
    float phase_inductance;
    float phase_resistance;
    osThreadId motor_thread;
    bool thread_ready;
    bool enable_control; // enable/disable via usb to start motor control. will be set to false again in case of errors.requires selftest_ok=true
    bool do_selftest; //  trigger motor selftest. will be reset to false after self test
    bool selftest_ok; 
    TIM_HandleTypeDef* motor_timer;
    uint16_t next_timings[3];
    uint16_t control_deadline;
    Iph_BC_t current_meas;
    Iph_BC_t DC_calib;
    DRV8301_Obj gate_driver;
    float shunt_conductance;
    Current_control_t current_control;
    Rotor_t rotor;
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
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc);
void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc);

//@TODO move motor thread to high level file
void motor_thread(void const * argument);

//@TODO move cmd parsing to high level file
void motor_parse_cmd(uint8_t* buffer, int len);

#endif //__LOW_LEVEL_H
