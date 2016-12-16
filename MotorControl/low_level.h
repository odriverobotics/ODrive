/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include "drv8301.h"

/* Exported types ------------------------------------------------------------*/
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
} Current_control_t;

typedef struct {
    osThreadId motor_thread;
    bool thread_ready;
    TIM_HandleTypeDef* motor_timer;
    TIM_HandleTypeDef* encoder_timer;
    int16_t encoder_offset;
    int32_t encoder_state;
    uint16_t next_timings[3];
    Iph_BC_t current_meas;
    Iph_BC_t DC_calib;
    DRV8301_Obj gate_driver;
    float shunt_conductance;
    Current_control_t current_control;
} Motor_t;

enum Motor_thread_signals {
    M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
};

/* Exported constants --------------------------------------------------------*/
extern float vbus_voltage;
extern Motor_t motors[];
extern const int num_motors;

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void safe_assert(int arg);
void init_motor_control();
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc);
void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc);

//@TODO move motor thread to high level file
void motor_thread(void const * argument);

#endif //__LOW_LEVEL_H
