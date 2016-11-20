
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

#include <cmsis_os.h>
#include "drv8301.h"

typedef struct Motor_s {
	TIM_HandleTypeDef* timer_handle;
	osMailQId* current_meas_queue;
    DRV8301_Obj gate_driver;
    float shunt_conductance;
    float maxcurrent;
} Motor_t;

extern Motor_t motor_configs[];
extern const int num_motors;

void init_motor_control();

//@TODO move motor thread to high level file
void motor_thread(void const * argument);

#endif //__LOW_LEVEL_H
