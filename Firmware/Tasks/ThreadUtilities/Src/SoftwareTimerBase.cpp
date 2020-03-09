/*******************************************************************************
* File          : SoftwareTimerBase.cpp
*
* Description   : Software timer utilities to create, start, and stop a software
*               timer and to set a callback function.
*
* Project       : 
*
* Author        : s.gilbert
*
* Created on    : 5 Jan 2020
*
*******************************************************************************/

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "SoftwareTimerBase.hpp"
#include "stdint.h"

/*******************************************************************************
NAMESPACE
*******************************************************************************/

using namespace STimerCore;
using namespace threadCore;

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

#define PERIOD_US2CLK(x)   (uint64_t)((float)x / (float)(portTICK_PERIOD_MS * 1000))

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

/**\brief   Constructor. Creates a static software timer.
 *
 * \param   periodUS    - timer period in US
 * \param   pFunction   - pointer to the function to call
 *
 * \return  None
 */
CSoftwareTimerBase::CSoftwareTimerBase(uint64_t periodUS, CThreadBase * const pFunction)
    : m_periodUS(periodUS)
    , m_pFunction(pFunction)
    , m_pSoftwareTimer(nullptr)
{
    // if no function pointer is provided then assume its part of a class
    if(nullptr == pFunction)
    {
        m_pFunction = (threadCore::CThreadBase * )this;
    }
}

/**\brief   Static callback function for the software timer. Calls the child
 *          class's run function.
 *
 * \param   timerID - Identifier of software timer
 *
 * \return  None
 */
void CSoftwareTimerBase::_callback(void * timerID)
{
    if (nullptr != timerID)
    {
        (reinterpret_cast<CSoftwareTimerBase *>(pvTimerGetTimerID(timerID)))->run();
    }
}

/**\brief   Starts the timer.
 *
 * \param   periodUS    - Period of the timer in microseconds
 * \param   pName       - Pointer to the string containing the name
 *
 * \return  None
 */
void CSoftwareTimerBase::startTimer(const int64_t periodUS, char const * pName)
{
    /* default argument is -1 so if we see it here, then the user probably
     * called startTimer with no arguments so don't update the timer period
     */
    if(-1 != periodUS)
    {
        m_periodUS = periodUS;
    }

    m_pSoftwareTimer = xTimerCreateStatic( pName,
                                           PERIOD_US2CLK(m_periodUS),
                                           pdTRUE,
                                           this,
                                           (TimerCallbackFunction_t)_callback,
                                           &m_STBuffer);

    (void)xTimerStart(m_pSoftwareTimer, PERIOD_US2CLK(m_periodUS));
}
