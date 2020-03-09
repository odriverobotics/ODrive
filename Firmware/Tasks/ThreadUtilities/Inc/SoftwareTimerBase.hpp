/*******************************************************************************
* File          : SoftwareTimerBase.hpp
*
* Description   : 
*
* Project       : 
*
* Author        : s.gilbert
*
* Created on    : 7 Jan 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SOFTWARETIMERBASE_HPP
#define SOFTWARETIMERBASE_HPP

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "ThreadBase.hpp"
#include "timers.h"

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

namespace STimerCore
{

/*
 * Platform dependent class for software timer based periodic callbacks
 */
class CSoftwareTimerBase
{
protected:
    CSoftwareTimerBase(uint64_t m_periodUS = 1000000
                       , threadCore::CThreadBase * const pFunction = nullptr);
    ~CSoftwareTimerBase() = default;
    void startTimer(const int64_t periodUS = -1
                    , char const * pName = "PeriodTimer");
    virtual void run() = 0;
    static void _callback(void * timerID);

protected:
    uint64_t m_periodUS;
    threadCore::CThreadBase * m_pFunction;
    TimerHandle_t m_pSoftwareTimer;
    StaticTimer_t m_STBuffer;
};

/*******************************************************************************
INLINE FUNCTION DEFINITIONS
*******************************************************************************/

} /* namespace STimerCore */

#endif /* SOFTWARETIMERBASE_HPP */
