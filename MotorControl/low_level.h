/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include "drv8301.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    M_SIGNAL_PH_CURRENT_MEAS = 1u << 0,
    M_SIGNAL_USB_MOTOR_CONTROL = 1u << 1
} Motor_thread_signals_t;

typedef enum {
    CURRENT_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
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
} Current_control_t;

typedef struct {
    TIM_HandleTypeDef* encoder_timer;
    int16_t encoder_offset;
    int32_t encoder_state;
    float phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;
} Rotor_t;

#define TIMING_LOG_SIZE 16
typedef struct {
    Motor_control_mode_t control_mode;
    float pos_setpoint;
    float pos_gain;
    float vel_setpoint;
    float vel_gain;
    float vel_limit;
    float current_setpoint;
    osThreadId motor_thread;
    bool thread_ready;
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

/* Exported constants --------------------------------------------------------*/
extern float vbus_voltage;
extern Motor_t motors[];
extern const int num_motors;
extern uint8_t pending_usb_buf[64];
extern osThreadId usb_mc_thread_id;

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
void usb_mc_thread(void const * argument);

#endif //__LOW_LEVEL_H
