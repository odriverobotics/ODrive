/*******************************************************************************
* File          : ThreadBase.hpp
*
* Description   : C++ encapsulated thread utilities tailored to run on FreeRTOS
*
* Project       :
*
* Author        : S.Gilbert
*
* Created on    : 5 Jan 2020
*
*******************************************************************************/

/******************************************************************************
INCLUDES
*******************************************************************************/

#include "ThreadBase.hpp"
#include "string.h"

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
INTERNAL FUNCTION DEFINTIONS
*******************************************************************************/

/*******************************************************************************
FUNCTION DECLARATIONS
*******************************************************************************/

/**\brief   Constructor.
 *
 * \param   pTaskName   - pointer to the task name
 * \param   priority    - task priority
 * \param   pStack      - pointer to stack space
 * \param   stackSize   - stack size in bytes
 * \return  None
 */
CThreadBase::CThreadBase(char const * const pTaskName
                         , const int32_t priority
                         , void * const pStack
                         , const size_t stackSize)
    : m_thread(this)
    , m_isRunning(false)
    , m_priority(priority)
    , m_pThreadHandle(nullptr)
    , m_pAlignedStack((StackType_t *)pStack)
    , m_stackSize(stackSize)
{
    memcpy(m_taskName, pTaskName, strlen(pTaskName));
}

/**\brief   Destructor. Stops the task, in case it was running.
 *
 * \param   None
 *
 * \return  None
 */
CThreadBase::~CThreadBase(void)
{
    deleteTask();
}

/**\brief   Starts the task.
 *
 * \param   priority    - priority of the task
 * \param   pTaskName   - pointer to the buffer containing the task name
 *
 * \return  true if task has been created and added to the scheduler
 */
bool CThreadBase::start(const int32_t priority, char const * pTaskName)
{
    if (nullptr != pTaskName)
    {
        memcpy(m_taskName, pTaskName, strlen(pTaskName));
    }

    /* if the priority has been explicitly defined in the argument list (assume
     * -1 means the user hasn't populated the argument), then update the member
     * variable to the one declared otherwise use the one defined in the
     * constructor. If one hasn't been defined in the constructor, it should
     * default to the lowest priority.
     */
    if (-1 < priority)
    {
        m_priority = priority;
    }

    m_pThreadHandle = createTask(m_taskName, m_priority);

    // if the thread handle is null, the task hasn't been created
    if(nullptr != m_pThreadHandle)
    {
        vTaskResume(m_pThreadHandle);
        m_isRunning = true;
    }

    return m_isRunning;
}

/**\brief   Stops the task by suspending it in the scheduler.
 *
 * \param   None
 *
 * \return  None
 */
void CThreadBase::stop(void)
{
    if (m_isRunning)
    {
        m_isRunning = false;
    }

    vTaskSuspend(m_pThreadHandle);
}

/**\brief   Suspends the task but leaves it listed as blocked in the scheduler.
 *
 * \param   None
 *
 * \return  None
 */
void CThreadBase::suspend(void)
{
    if (m_isRunning)
    {
        m_isRunning = false;
        m_thread.suspend();
    }
}

/**\brief   Resumes the task from the scheduler suspend state.
 *
 * \param   None
 *
 * \return  None
 */
void CThreadBase::resume(void)
{
    vTaskResume(m_pThreadHandle);
    m_isRunning = true;
}

/**\brief   Stops the task and removes it from the scheduler.
 *
 * \param   None
 *
 * \return  None
 */
void CThreadBase::deleteTask(void)
{
    stop();
    if(nullptr != m_pThreadHandle)
    {
        vTaskDelete(m_pThreadHandle);
    }
}

/**\brief   Puts the task to sleep for a defined period.
 *
 * \param   milliseconds    - length of time to put the task to sleep
 *
 * \return  None
 */
void CThreadBase::sleepMilliseconds(const uint32_t milliseconds)
{
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

/**\brief   Returns the running state of the task.
 *
 * \param   None
 *
 * \return  Running state of the task
 */
bool CThreadBase::isRunning(void)
{
    return m_isRunning;
}

/**\brief   Sets the name of the task.
 *
 * \param   pName   - pointer to the buffer containing the task name
 *
 * \return  None
 */
void CThreadBase::setName(char const * const pName)
{
    // TODO - find a way to populate the name of a the RTOS task
//    if (nullptr != m_pThreadHandle)
//    {
//        m_pThreadHandle->name = pName;
//    }
}

/**\brief   Gets the name of the task.
 *
 * \param   None
 *
 * \return  pointer to task name
 */
char const * CThreadBase::getName(void)
{
    return m_taskName;
}

/**\brief   Returns task pointer.
 *
 * \param   None
 *
 * \return  returns task pointer
 */
threadHandle_t CThreadBase::getHandle(void)
{
  return this->m_pThreadHandle;
}

/**\brief   Single call function stub before main task is called.
 *
 * \param   None
 *
 * \return  None
 */
void CThreadBase::begin(void)
{

}

/**\brief   Single call function stub after main task is exited.
 *
 * \param   None
 *
 * \return  None
 */
void CThreadBase::end(void)
{

}

/**\brief   Static function to allow calling of the class instance's main
 *          function.
 *
 * \param   pArg    - pointer to the class instance
 *
 * \return  None
 */
void CThreadBase::_ThdFunc(void * pArg)
{
    if(nullptr != pArg)
    {
        CThreadBase * pThread = (CThreadBase*)pArg;
        pThread->begin();
        pThread->threadFunc();
        pThread->end();
    }
}

/**\brief   Creates a task and inserts it in to the scheduler.
 *
 * \param   pTaskName   - pointer to the buffer containing the task name
 * \param   priority    - priority of the task
 *
 * \return  None
 */
threadHandle_t CThreadBase::createTask(char const * pTaskName, const uint32_t priority)
{
    threadHandle_t xHandle = nullptr;
// todo consider using osThreadCreate as it'll push the dynamic/static allocation to CMSIS_OS.h
#if( configSUPPORT_STATIC_ALLOCATION == 1 )

    if ((nullptr != m_pAlignedStack) && (0 != m_stackSize))
    {
        xHandle = xTaskCreateStatic(this->_ThdFunc,
                                    pTaskName,
                                    m_stackSize / sizeof(&m_pAlignedStack[0]),
                                    this,
                                    priority,
                                    m_pAlignedStack,
                                    &m_xTaskBuffer);
    }

#endif

// TODO - fill in dynamic allocation option
//#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
//
//    xTaskCreate(this->_ThdFunc,
//                pName,
//                m_stackSize / sizeof(uint32_t),
//                this,
//                priority,
//                &xHandle);
//    configASSERT( xHandle );
//
//#endif

    vTaskSuspend(m_pThreadHandle);

    return xHandle;

}
