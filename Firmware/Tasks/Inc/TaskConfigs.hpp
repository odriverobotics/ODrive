/*******************************************************************************
* File          : TaskConfigs.hpp
*
* Description   : 
*
* Project       :
*
* Author        : s.gilbert
*
* Created on    : 10 Feb 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TASKCONFIGS_HPP
#define TASKCONFIGS_HPP

/******************************************************************************
INCLUDES
*******************************************************************************/

#include <stdint.h>
#include <cmsis_os.h>

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

/*******************************************************************************
TYPES
*******************************************************************************/

typedef struct
{
    char * pThreadName;
    uint32_t stackSize;
    osPriority priority;
}taskStruct_t;

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
CONSTANTS
*******************************************************************************/


/* stack size is number of 32 bit words taken from the RTOS heap currently set
 * to 65536 in cubeMX
 */
constexpr taskStruct_t cmd_parse        {(char*)"CMDParser",           8096, osPriorityNormal};
constexpr taskStruct_t UART_server      {(char*)"UARTServer",          1024, osPriorityNormal};
constexpr taskStruct_t USB_server       {(char*)"USBServer",           1024, osPriorityNormal};
constexpr taskStruct_t Axis_thread      {(char*)"AxisServer",          2048, osPriorityNormal};
constexpr taskStruct_t Analog_server    {(char*)"AnalogServer",        2048, osPriorityLow};
constexpr taskStruct_t DebounceTask     {(char*)"DebounceTask",        128, osPriorityNormal};

/*******************************************************************************
NAMESPACE
*******************************************************************************/

/*******************************************************************************
INLINE FUNCTION DEFINITIONS
*******************************************************************************/


#endif /* TASKCONFIGS_HPP */
