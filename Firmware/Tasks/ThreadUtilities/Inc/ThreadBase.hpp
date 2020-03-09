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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef THREADBASE_HPP
#define THREADBASE_HPP

/******************************************************************************
INCLUDES
*******************************************************************************/

#include "ThreadControl.hpp"

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

typedef TaskHandle_t threadHandle_t;

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

class CThreadBase
{
public:
    CThreadBase(char const * const pTaskName = "Anon Task"
                , const int32_t priority = 1
                , void * const pStack = nullptr
                , const size_t stackSize = 0u);
    virtual ~CThreadBase(void);
    virtual void begin(void);
    virtual void end(void);
    virtual void threadFunc(void) = 0;
    bool start(const int32_t priority = -1, char const * pTaskName = nullptr);
    void stop(void);
    void suspend(void);
    void resume(void);
    void deleteTask(void);
    bool isRunning(void);
    virtual void sleepMilliseconds(const uint32_t milliseconds);
    void setName(char const * const name);
    char const * getName(void);
    threadHandle_t getHandle(void);

private:
    static void _ThdFunc(void * pArg);
    threadHandle_t createTask(char const * pTaskName, const uint32_t priority);

protected:
    CThreadControl m_thread;
    volatile bool m_isRunning;
    char m_taskName[128];
    uint32_t m_priority;
    threadHandle_t m_pThreadHandle;
    StackType_t * m_pAlignedStack;
    size_t m_stackSize;
    StaticTask_t m_xTaskBuffer;
};

} /* namespace threadCore */

#endif /* THREADBASE_HPP */
