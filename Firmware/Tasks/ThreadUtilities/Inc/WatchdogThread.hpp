/*******************************************************************************
* File          : WatchdogThread.hpp
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
#ifndef WATCHDOGTHREAD_HPP
#define WATCHDOGTHREAD_HPP

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "stdint.h"

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

/*******************************************************************************
TYPES
*******************************************************************************/

/**\brief   Function to be called when main task times out.
 *
 * \param       - class instance pointer
 * \param       - message pointer
 *
 * \return  true if no restart is needed, false if watchdog restart needed
 */
typedef bool (* timeoutFunc)(void *, void *);

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
NAMESPACE
*******************************************************************************/

class CWatchdogBase
{
public:
    CWatchdogBase() = default;
    virtual ~CWatchdogBase() = default;
    virtual int32_t registerTask(void * pThreadHandle
                                 , uint32_t periodMS
                                 , timeoutFunc callback = nullptr
                                 , void * pInstance = nullptr
                                 , void * pMsg = nullptr) = 0;
    virtual int32_t deregisterTask(void * pThreadHandle) = 0;
    virtual int32_t feed(void * pThreadHandle) = 0;
    virtual int32_t sleep(void * pThreadHandle) = 0;
};

/*******************************************************************************
INLINE FUNCTION DEFINITIONS
*******************************************************************************/


#endif /* WATCHDOGTHREAD_HPP */
