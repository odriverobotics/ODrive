/*******************************************************************************
* File          : ThreadControl.hpp
*
* Description   : Class to define basic thread control utilities.
*
* Project       : 
*
* Author        : s.gilbert
*
* Created on    : 1 Jan 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef THREADCONTROL_HPP
#define THREADCONTROL_HPP

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

/*******************************************************************************
TYPES
*******************************************************************************/

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
NAMESPACE
*******************************************************************************/

namespace threadCore
{

class CThreadControl
{
public:
    CThreadControl(TaskHandle_t pThread = nullptr);
    virtual ~CThreadControl() = default;
    void setThread(TaskHandle_t pThread);
    TaskHandle_t getThread(void);
    virtual void resume(uint32_t msg = 0);
    virtual void resumeFromISR(uint32_t msg = 0);
    virtual uint32_t suspend(uint32_t period = portMAX_DELAY);
    virtual uint32_t suspendFromISR(void);

private:
    TaskHandle_t m_pThread;
    SemaphoreHandle_t xSemaphore;
    StaticSemaphore_t xSemaphoreBuffer;
};

/*******************************************************************************
INLINE FUNCTION DEFINITIONS
*******************************************************************************/

} /* namespace threadCore */

#endif /* THREADCONTROL_HPP */
