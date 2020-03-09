/*******************************************************************************
* File          : Thread.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef THREAD_HPP
#define THREAD_HPP

/******************************************************************************
INCLUDES
*******************************************************************************/

#include <stddef.h>
#include "SoftwareTimerBase.hpp"
#include "ThreadTemplate.hpp"

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

template <typename ObjT>
class CThread
        : public STimerCore::CSoftwareTimerBase
        , public CThreadTemplate<ObjT>
{
public:
    CThread(uint64_t periodUS = 1000000, typename CThreadTemplate<ObjT>::FuncT func = nullptr, ObjT * const pObj = nullptr);
    virtual ~CThread(void) = default;
    void startPeriodic(const uint64_t periodUS, char const * pName = "Thread periodic timer");
    void threadFunc(void) override;
    virtual void begin(void);
    virtual void end(void);
    virtual void run(void) override;

private:
    void start(const uint32_t priority = 0);
    void start(const char* pName, const uint32_t priority = 0);
};

/*******************************************************************************
INLINE FUNCTIONS
*******************************************************************************/

/**\brief   Constructor. Sets up the timer and the thread like function.
 *
 * \param   periodUS    - period in microseconds
 * \param   func        - primary function to be repetatively called
 * \param   pObj        - pointer to the class containing func
 *
 * \return  None
 */
template <typename ObjT>
CThread<ObjT>::CThread(const uint64_t periodUS, typename CThreadTemplate<ObjT>::FuncT func, ObjT * const pObj)
    : CSoftwareTimerBase(periodUS, this)
    , CThreadTemplate<ObjT>(func, pObj)
{}

/**\brief   Starts the periodic task or function.
 *
 * \param   periodUS    - period in microseconds
 * \param   pName       - pointer to task name string
 *
 * \return  None
 */
template <typename ObjT>
void CThread<ObjT>::startPeriodic(const uint64_t periodUS, char const * pName)
{
    STimerCore::CSoftwareTimerBase::startTimer((int64_t)periodUS, pName);
}

/**\brief   Basic wrapper to allow repetitive calls until run flag is called.
 *
 * \param   None
 *
 * \return  None
 */
template <typename ObjT>
void CThread<ObjT>::threadFunc(void)
{
    begin();
    while (this->m_isRunning)
    {
        run();
    }
    end();
    this->m_thread.resume();
}

/**\brief   Single call function stub before main task is called.
 *
 * \param   None
 *
 * \return  None
 */
template <typename ObjT>
void CThread<ObjT>::begin(void)
{

}

/**\brief   Single call function stub after main task is exited.
 *
 * \param   None
 *
 * \return  None
 */
template <typename ObjT>
void CThread<ObjT>::end(void)
{

}

/**\brief   Calls the 'begin', 'm_func' and 'end' routines in sequence.
 *
 * \param   None
 *
 * \return  None
 */
template <typename ObjT>
void CThread<ObjT>::run(void)
{
    (CThreadTemplate<ObjT>::m_pObj->*CThreadTemplate<ObjT>::m_func)();
}

/**\brief   Creates and starts the task.
 *
 * \param   priority    - priority of the task
 *
 * \return  None
 */
template <typename ObjT>
void CThread<ObjT>::start(const uint32_t priority)
{
    CThreadTemplate<ObjT>::start(priority);
}

/**\brief   Creates and starts the task.
 *
 * \param   pName       - pointer to task name string
 * \param   priority    - priority of the task
 *
 * \return  None
 */
template <typename ObjT>
void CThread<ObjT>::start(const char * pName, const uint32_t priority)
{
    CThreadTemplate<ObjT>::start(pName, priority);
}

} /* namespace threadCore */

#endif /* THREAD_HPP ------------------------------------------------------*/
