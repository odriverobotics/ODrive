#include <cmsis_os.h>
#include <freertos_vars.h>
#include <stdio.h>
#include <myapp.h>
#include "odrive_main.h"


TaskHandle_t MYAPPTaskHandle=NULL;
void StartMYAPPTask(void const * argument);
extern std::array<Axis, AXIS_COUNT> axes;
extern ODrive odrv;

void start_MYAPP(void)
{
    xTaskCreate((TaskFunction_t )StartMYAPPTask,
              (const char*)"MYAPPTask",
              (uint16_t)128,
              (void*)NULL,
              (UBaseType_t)0,
              (TaskHandle_t*)&MYAPPTaskHandle
              );
}


void StartMYAPPTask(void const * argument)
{
    Axis& axis0 = axes[0];
    Axis& axis1 = axes[1];
    int cnt=0;
    if(!axis0.motor_.config_.pre_calibrated)
    {
        axis0.motor_.config_.current_lim=4.0f;
        axis0.controller_.config_.vel_limit=10.0f;
        odrv.config_.brake_resistance=0.01f;
        axis0.motor_.config_.pole_pairs=20;
        axis0.motor_.config_.torque_constant=0.0827f;
        axis0.motor_.config_.motor_type=Motor::MOTOR_TYPE_HIGH_CURRENT;
        axis0.encoder_.config_.cpr=16384;
        axis0.controller_.config_.pos_gain=50.0f;
        axis0.controller_.config_.vel_gain=0.25f;
        axis0.controller_.config_.vel_integrator_gain=0.6f;
        axis0.requested_state_=Axis::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
        axis0.config_.startup_encoder_offset_calibration=1;
        axis0.config_.startup_closed_loop_control=1;
        axis0.motor_.config_.pre_calibrated=1;
        vTaskDelay(15000);
        axis1.motor_.config_.current_lim=4.0f;
        axis1.controller_.config_.vel_limit=20.0f;
        axis1.motor_.config_.pole_pairs=7;
        axis1.motor_.config_.torque_constant=0.0197f;
        axis1.motor_.config_.motor_type=Motor::MOTOR_TYPE_HIGH_CURRENT;
        axis1.encoder_.config_.cpr=2000;
        axis1.controller_.config_.pos_gain=100.0f;
        axis1.controller_.config_.vel_gain=0.008f;
        axis1.controller_.config_.vel_integrator_gain=0.04f;
        axis1.requested_state_=Axis::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
        axis1.config_.startup_encoder_offset_calibration=1;
        axis1.config_.startup_closed_loop_control=1;
        axis1.motor_.config_.pre_calibrated=1;
        vTaskDelay(15000);
        odrv.save_configuration();
        odrv.reboot();
    }
    axis1.controller_.config_.control_mode=Controller::CONTROL_MODE_TORQUE_CONTROL;
    for(;;)
    {
        
        osSemaphoreWait(sem_my,0);
        if(cnt==7)
        {
            cnt=0;
            axis0.controller_.input_pos_=-1.0f*axis1.encoder_.pos_estimate_;
            axis1.controller_.input_torque_=0.1f*axis0.controller_.torque_out_;
        }
        else
            cnt+=1;
        //osThreadSuspend(nullptr);
        //vTaskDelay(1);
    }
}

void resume_MYAPP(void* arg)
{
    osThreadResume(MYAPPTaskHandle);
}
