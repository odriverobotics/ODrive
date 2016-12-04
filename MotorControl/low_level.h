
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

#include <cmsis_os.h>
#include "drv8301.h"

typedef struct {
    float phB;
    float phC;
} Iph_BC_t;

typedef struct Motor_s {
    osThreadId motor_thread;
    TIM_HandleTypeDef* timer_handle;
    Iph_BC_t current_meas;
    DRV8301_Obj gate_driver;
    float shunt_conductance;
    float maxcurrent;
} Motor_t;

enum Motor_thread_signals {
    M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
};

extern Motor_t motors[];
extern const int num_motors;

void init_motor_control();

//@TODO move motor thread to high level file
void motor_thread(void const * argument);

#endif //__LOW_LEVEL_H
