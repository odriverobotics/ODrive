/*******************************************************************************
* File          : TaskBase.hpp
*
* Description   : 
*
* Project       : 
*
* Author        : s.gilbert
*
* Created on    : 03 Jan 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TASKBASE_HPP
#define TASKBASE_HPP

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include "Thread.hpp"
#include "WatchdogThread.hpp"

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

/* converts task sample period to system clock counts */
#define SAMPLE2CLK(x) (uint32_t)(((float)configTICK_RATE_HZ * 1000.0f) / (float)x)

#ifndef ARRAY_LEN
#define ARRAY_LEN(x) (sizeof(x) / sizeof(x[0]))
#endif

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

class CTaskBase
        : public CThreadBase
{
public:
    CTaskBase(char const * const pTaskName
              , float frequency
              , uint32_t priority
              , void * const pStack
              , const size_t stackSize
              , CWatchdogBase * pWatchdog = nullptr
              , int32_t m_timeoutMS = 10);
    virtual ~CTaskBase(void) = default;
    void setTaskFrequency(float freq = 0.0f);
    void timerCallback(void);
    void begin(void) override;
    virtual void funcBegin(void);
    void threadFunc(void) override;
    void end(void) override;
    virtual void funcEnd(void);
    virtual bool funcTimeout(void * pMsg);
    virtual void funcMain(void) = 0;
    void suspendTask(void);
    void resumeTask(void);
    void resumeTaskFromISR(void);

private:
    static bool _funcTimeout(void * pInstance, void * pMsg);

private:
    uint32_t m_taskPeriodMS;
    CThread<CTaskBase> m_timer;
    CWatchdogBase * m_pWatchdog;
    int32_t m_timeoutMS;
    void * m_pTimeoutMsg;
};

/*******************************************************************************
INLINE FUNCTIONS
*******************************************************************************/

/**\brief   Constructor
 *
 * \param   pTaskName   - pointer to the task name
 * \param   frequency   - task frequency
 * \param   priority    - task priority
 * \param   pStack      - pointer to stack space
 * \param   stackSize   - stack size in bytes
 * \param   pWatchdog   - pointer to the watchdog class
 * \param   timeoutMS   - task timeout period in MS
 *
 * \return  None
 */
inline CTaskBase::CTaskBase(char const * const pTaskName
                            , float frequency
                            , uint32_t priority
                            , void * const pStack
                            , const size_t stackSize
                            , CWatchdogBase * pWatchdog
                            , int32_t timeoutMS)
    : CThreadBase(pTaskName, priority, pStack, stackSize)
    , m_taskPeriodMS((frequency > 0.0f) ? SAMPLE2CLK(frequency) : 0u)
    , m_timer(m_taskPeriodMS, &CTaskBase::timerCallback, this)
    , m_pWatchdog(pWatchdog)
    , m_timeoutMS(timeoutMS)
    , m_pTimeoutMsg(nullptr)
{}

/**\brief   Sets the task frequency.
 *`
 * \param   freq    - task frequency in Hz
 *
 * \return  None
 */
inline void CTaskBase::setTaskFrequency(float freq)
{
    m_taskPeriodMS = (freq > 0.0f) ? SAMPLE2CLK(freq) : 0u;
}

/**\brief   Timer callback, wakes the associated task
*
* \param    None
*
* \return   None
*/
inline void CTaskBase::timerCallback(void)
{
    this->resumeTaskFromISR();
}

/**\brief   Single call function stub before main task is called. Registers
 *          watchdog if one is provided and, starts periodic timer if period is
 *          non-zero and finally calls funcBegin()
 *
 * \param   None
 *
 * \return  None
 */
inline void CTaskBase::begin(void)
{
    if(nullptr != m_pWatchdog)
    {
        m_pWatchdog->registerTask(this->getHandle(), m_timeoutMS, _funcTimeout, this);
    }

    if(0 < m_taskPeriodMS)
    {
        m_timer.startPeriodic(m_taskPeriodMS);
    }

    funcBegin();
}

/**\brief   Single call function stub before main task is called designed to
 *          allow a task to call one shot functions before it's main is envoked
 *
 * \param   None
 *
 * \return  None
 */
inline void CTaskBase::funcBegin(void)
{

}

/**\brief   Base thread function. Services watchdog, calls task main, and
 *          suspends task to be woken by either software timer or user code
 *          (or a mix of both) calling resume.
 *
 * \param   None
 *
 * \return  None
 */
inline void CTaskBase::threadFunc(void)
{
    while (m_isRunning)
    {
        if(nullptr != m_pWatchdog)
        {
            this->m_pWatchdog->feed(this->getHandle());
        }

        funcMain();

        if(nullptr != m_pWatchdog)
        {
            m_pWatchdog->sleep(this->getHandle());
        }

        this->suspendTask();
    }
}

/**\brief   Single call function stub after main task is exited.
 *
 * \param   None
 *
 * \return  None
 */
inline void CTaskBase::end(void)
{
    if(nullptr != m_pWatchdog)
    {
        m_pWatchdog->deregisterTask(this->getHandle());
    }

    funcEnd();
}

/**\brief   Single call function stub after main task has returned, designed to
 *          allow a task to call one shot functions after it's main has exited
 *
 * \param   None
 *
 * \return  None
 */
inline void CTaskBase::funcEnd(void)
{

}

/**\brief   Single call function stub after main task has triggered the watchdog
 *          monitoring task timeout but before the system reset occurs. Designed
 *          to allow a task to call one shot functions after it's main task has
 *          locked up. This particular instance of the routine will convert
 *          the contents of the message to a bool allowing the user to specify
 *          whether the system should restart or not on timeout without having
 *          to explicitly say so.
 *
 * \param   pMsg    - pointer to a message for the timeout function
 *
 * \return  true if no restart required, else returns false
 */
inline bool CTaskBase::funcTimeout(void * pMsg)
{
    bool returnVal = false;

    if(nullptr == pMsg)
    {
        returnVal = *(bool *)pMsg;
    }

    return returnVal;
}

/**\brief   Suspend function, allows suspending of task. Cannot be called from
 *          inside an ISR
 *
 * \param   None
 *
 * \return  None
 */
inline void CTaskBase::suspendTask(void)
{
    m_thread.suspend();
}

/**\brief   Resume function, allows waking up of task. Cannot be called from
 *          inside an ISR
 *
 * \param   None
 *
 * \return  None
 */
inline void CTaskBase::resumeTask(void)
{
    m_thread.resume();
}

/**\brief   Resume function, allows waking up of task from inside an ISR
 *
 * \param   None
 *
 * \return  None
 */
inline void CTaskBase::resumeTaskFromISR(void)
{
    m_thread.resumeFromISR();
}

/**\brief   Thread timeout static function. Allows call of Virtual class timeout
 *          function which can be overridden by the child class
 *
 * \param   pInstance   - pointer to the created object
 * \param   pMsg        - pointer to the message
 *
 * \return  true if no restart required, else returns false
 */
inline bool CTaskBase::_funcTimeout(void * pInstance, void * pMsg)
{
    bool returnVal = false;

    if(nullptr != pInstance)
    {
        returnVal = ((CTaskBase *)pInstance)->funcTimeout(pMsg);
    }

    return returnVal;
}

} /* namespace threadCore */

#endif /* TASKBASE_HPP -------------------------------------------------------*/
