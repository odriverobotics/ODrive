
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM_INPUT_H
#define __PWM_INPUT_H

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

// Initalisation
void pwm_in_init();

// called from STM platform code
extern "C" {
void pwm_in_cb(int channel, uint32_t timestamp);
}

#ifdef __cplusplus
}
#endif

#endif //__PWM_INPUT_H
