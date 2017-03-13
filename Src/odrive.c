#include "FreeRTOS.h"
#include "task.h"

#include "freertos_vars.h"
#include "low_level.h"
#include "version.h"

#include "../MotorControl/low_level.h"


//TODO: Refactor out to own odrive_usb.c/h
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void usb_cmd_thread(void const * argument) {

	for (;;) {
		// Wait for signalling from USB interrupt (OTG_FS_IRQHandler)
		osSemaphoreWait(sem_usb_irq, osWaitForever);
		// Irq processing loop
		//while(HAL_NVIC_GetActive(OTG_FS_IRQn)) {
		HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
		//}
		// Let the irq (OTG_FS_IRQHandler) fire again.
		HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
	}

	// If we get here, then this task is done
	vTaskDelete(osThreadGetId());
}

void ODriveSemaphoreInit() {
	osSemaphoreDef(sem_usb_irq);
	sem_usb_irq = osSemaphoreCreate(osSemaphore(sem_usb_irq), 1);
	osSemaphoreWait(sem_usb_irq, 0);

}

void ODriveDefaultTask(void const * argument) {
	// Init motor control
	init_motor_control();

	// Start motor threads
	osThreadDef(task_motor_0, motor_thread, osPriorityHigh + 1, 0, 512);
	osThreadDef(task_motor_1, motor_thread, osPriorityHigh, 0, 512);
	thread_motor_0 = osThreadCreate(osThread(task_motor_0), &motors[0]);
	thread_motor_1 = osThreadCreate(osThread(task_motor_1), &motors[1]);

	// Start USB command handling thread
	osThreadDef(task_usb_cmd, usb_cmd_thread, osPriorityNormal, 0, 512);
	thread_usb_cmd = osThreadCreate(osThread(task_usb_cmd), NULL);

	//If we get to here, then the default task is done.
	vTaskDelete(NULL);
}


