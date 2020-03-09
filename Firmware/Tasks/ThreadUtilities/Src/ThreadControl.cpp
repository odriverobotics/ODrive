/*******************************************************************************
* File          : ThreadControl.cpp
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

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "ThreadControl.hpp"

/*******************************************************************************
NAMESPACE
*******************************************************************************/

using namespace threadCore;

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
MODULE VARIABLES
*******************************************************************************/

/*******************************************************************************
MODULE FUNCTION DECLARATIONS
*******************************************************************************/

/*******************************************************************************
FUNCTION DEFINITIONS
*******************************************************************************/

/**\brief   Constructor.
 *
 * \param   pThread - pointer to the thread this class can control
 *
 * \return  None
 */
CThreadControl::CThreadControl(TaskHandle_t pThread)
    : m_pThread(pThread)
    , xSemaphore(nullptr)
{
    xSemaphore = xSemaphoreCreateBinaryStatic(&xSemaphoreBuffer);
    configASSERT(xSemaphore);
}

/**\brief   Sets the associated thread handle.
 *
 * \param   pThread - pointer to the thread this class can control
 *
 * \return  None
 */
void CThreadControl::setThread(TaskHandle_t pThread)
{
    m_pThread = pThread;
}

/**\brief   Gets the associated thread handle.
 *
 * \param   None
 *
 * \return  pointer to the thread this class can control
 */
TaskHandle_t CThreadControl::getThread(void)
{
    return m_pThread;
}

/**\brief   Resumes the suspended task. Do not call from inside an ISR.
 *
 * \param   msg - message to send to the task
 *
 * \return  None
 */
void CThreadControl::resume(uint32_t msg)
{
    (void)msg;  // TODO - need to add a message queue to sent this value to the task
    xSemaphoreGive(xSemaphore);
}

/**\brief   Resumes the suspended task. Can be called from inside an ISR.
 *
 * \param   msg - message to send to the task
 *
 * \return  None
 */
void CThreadControl::resumeFromISR(uint32_t msg)
{
    (void)msg;  // TODO - need to add a message queue to sent this value to the task
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken != pdFALSE )
    {
        // We can force a context switch here.  Context switching from an
        // ISR uses port specific syntax.  Check the demo task for your port
        // to find the syntax required.
    }
}

/**\brief   Suspends a running task. Can be called from inside an ISR.
 *
 * \param   period  - period the task is to be suspended for. Leave blank or use
 *                  portMAX_DELAY to leave the task suspended until woken by
 *                  releasing the semaphore.
 *
 * \return  0
 */
uint32_t CThreadControl::suspend(uint32_t period)
{
    xSemaphoreTake(xSemaphore, period);

    return 0;
}

/**\brief   Suspends the running task. Can be called from inside an ISR.
 *
 * \param   None
 *
 * \return  0
 */
uint32_t CThreadControl::suspendFromISR(void)
{
    static BaseType_t xHigherPriorityTaskWoken;

    xSemaphoreTakeFromISR(xSemaphore, &xHigherPriorityTaskWoken);

    return 0;
}
