/*******************************************************************************
* File          : WatchdogTask.cpp
*
* Description   : 
*
* Project       : 
*
* Author        : s.gilbert
*
* Created on    : 3 Mar 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "WatchdogTask.hpp"

/*******************************************************************************
NAMESPACE
*******************************************************************************/


/*******************************************************************************
DEFINITIONS
*******************************************************************************/

#ifndef SOFTWARE_BREAKPOINT
#define SOFTWARE_BREAKPOINT __asm__("BKPT")
#endif

// set DEBUG to true to enable a software breakpoint to catch a watchdog timeout
#ifdef DEBUG
#define CATCH_WATCHDOG_EVENT SOFTWARE_BREAKPOINT
#else
#define CATCH_WATCHDOG_EVENT
#endif

/*******************************************************************************
TYPES
*******************************************************************************/

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
MODULE VARIABLES
*******************************************************************************/

uint8_t taskTable[MAX_TRACKERS * sizeof(CNode<activity_t>)];

/*******************************************************************************
MODULE FUNCTION DECLARATIONS
*******************************************************************************/

/*******************************************************************************
FUNCTION DEFINITIONS
*******************************************************************************/

/**\brief   Constructor
 *
 * \param   pName       - pointer to the task nametion
 * \param   freq        - task update frequency
 * \param   priority    - task priority
 * \param   pStack      - pointer to stack space
 * \param   stackSize   - stack size in bytes
 * \param   pWatchdog   - pointer to the watchdog hardware
 *
 * \return  None
 */
CWatchdogTask::CWatchdogTask(char const * const pName
                             , const float freq
                             , const int32_t priority
                             , void * const pStack
                             , const size_t stackSize
                             , IWDG_HandleTypeDef * pWatchdog)
    : threadCore::CTaskBase(pName, freq, priority, pStack, stackSize, nullptr)
    , m_pWatchdog(pWatchdog)
    , m_freq(freq)
    , m_activityTracker(taskTable, sizeof(taskTable))
    , m_feed(true)
{}

/**\brief   Registers a task with the watchdog for timeout tracking. The task
 *          will be marked as asleep so can be used to register a task that
 *          isn't yet running.
 *
 * \param   pHandle     - unique value associated to the task that's being
 *                      registered (usually pointer to that task)
 * \param   periodMS    - maximum period in MS the task will be active for
 * \param   callback    - pointer to function to call when task times out
 * \param   pInstance   - pointer to class instance
 * \param   pMsg        - pointer to a message for the timeout function
 *
 * \return  A handle of the allocated counter or FAIL
 */
int32_t CWatchdogTask::registerTask(void * pHandle
                                    , uint32_t periodMS
                                    , timeoutFunc callback
                                    , void * pInstance
                                    , void * pMsg)
{
    watchdogHandle_t returnVal = WD_FAIL;
    activity_t activityTracker;

    size_t tableCount = m_activityTracker.countNodes();
    for(auto i = 0u; i < tableCount; ++i)
    {
        m_activityTracker.peakFromNode(i, &activityTracker);

        if(activityTracker.taskHandle == pHandle)
        {
            m_activityTracker.popFromNode(i, &activityTracker);
            break;
        }
    }

    activityTracker.taskHandle = pHandle;
    activityTracker.periodMS = periodMS;
    activityTracker.sleep = true;
    activityTracker.maxCounter = periodMS;
    activityTracker.callback = callback;
    activityTracker.pInstance = pInstance;
    activityTracker.pMsg = pMsg;

    returnVal = m_activityTracker.pushToBack(&activityTracker);

    return returnVal;
}

/**\brief   Deregisters the task from the watchdog tracker list.
 *
 * \param   pHandle - unique ID associated to the task that's being deregistered
 *
 * \return  error state, WD_SUCCESS on success, WD_FAIL on fail
 */
int32_t CWatchdogTask::deregisterTask(void * pHandle)
{
    watchdogHandle_t returnVal = WD_FAIL;

    size_t tableCount = m_activityTracker.countNodes();
    for(auto i = 0u; i < tableCount; ++i)
    {
        activity_t activityTracker;
        m_activityTracker.peakFromNode(i, &activityTracker);

        if(activityTracker.taskHandle == pHandle)
        {
            m_activityTracker.popFromNode(i, &activityTracker);
            returnVal = WD_SUCCESS;
            break;
        }
    }

    return returnVal;
}

/**\brief   Resets the watchdog counter associated with the task handle. Also
 *          sets the tracker to indicate the task is no longer asleep.
 *
 * \param   Task    - the handle of the task
 *
 * \return  error state, WD_SUCCESS on success, WD_FAIL on fail
 */
int32_t CWatchdogTask::feed(void * pHandle)
{
    int32_t returnVal = WD_FAIL;

    size_t tableCount = m_activityTracker.countNodes();
    for(auto i = 0u; i < tableCount; ++i)
    {
        activity_t activityTracker;
        m_activityTracker.peakFromNode(i, &activityTracker);

        if(activityTracker.taskHandle == pHandle)
        {
            activityTracker.sleep = false;
            activityTracker.periodMS = activityTracker.maxCounter;
            m_activityTracker.updateNode(i, &activityTracker);
            returnVal = WD_SUCCESS;
            break;
        }
    }

    return returnVal;
}

/**\brief   Pauses the timeout tracking of a task
 *
 * \param   Task    - handle of the task to pause monitoring
 *
 * \return  error state, WD_SUCCESS on success, WD_FAIL on fail
 */
int32_t CWatchdogTask::sleep(void * pHandle)
{
    int32_t returnVal = WD_FAIL;

    size_t tableCount = m_activityTracker.countNodes();
    for(auto i = 0u; i < tableCount; ++i)
    {
        activity_t activityTracker;
        m_activityTracker.peakFromNode(i, &activityTracker);

        if(activityTracker.taskHandle == pHandle)
        {
            activityTracker.sleep = true;
            m_activityTracker.updateNode(i, &activityTracker);
            returnVal = WD_SUCCESS;
            break;
        }
    }

    return returnVal;
}

/**\brief   Starts up the watchdog
 *
 * \param   None
 *
 * \return  None
 */
void CWatchdogTask::funcBegin(void)
{
    m_pWatchdog->Instance = IWDG;
    m_pWatchdog->Init.Prescaler = IWDG_PRESCALER_4;
    m_pWatchdog->Init.Reload = 4095;
#ifdef DEBUG
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
    if (HAL_IWDG_Init(m_pWatchdog) != HAL_OK)
    {
//      Error_Handler();
    }

}

/**\brief   The task for monitoring the tasks
 *
 * \param   None
 *
 * \return  None
 */
void CWatchdogTask::funcMain(void)
{
    size_t tableCount = m_activityTracker.countNodes();

    for(auto i = 0u; i < tableCount; ++i)
    {
        activity_t activityTracker;
        m_activityTracker.peakFromNode(i, &activityTracker);

        if(!activityTracker.sleep)
        {
            activityTracker.periodMS -= (0 < activityTracker.periodMS) ? 1 : 0;   /* don't want an underflow condition */
            (void)m_activityTracker.updateNode(i, &activityTracker);

            /* if the counter has expired, check to see if there's a callback
             * and call it. If a callback is present, it should indicate 'true'
             * if its ok to continue without a system restart or 'false' if a
             * restart should occur.
             */
            if(0 < activityTracker.periodMS)
            {
                /* If a callback is found, call it and 'AND' result with m_feed.
                 * otherwise set m_feed to false. Prevents setting m_feed to
                 * true when another task has stalled and requires a system
                 * reset.
                 */
                if(activityTracker.callback)
                {
                    m_feed = activityTracker.callback(activityTracker.pInstance, activityTracker.pMsg)
                            && (m_feed == true);
                }
                else
                {
                    m_feed = false;
                }
            }
        }
    }

    if(m_feed)
    {
        HAL_IWDG_Refresh(m_pWatchdog);
    }
    else
    {
        CATCH_WATCHDOG_EVENT;
    }
}
