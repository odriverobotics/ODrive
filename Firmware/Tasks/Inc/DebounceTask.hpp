/*******************************************************************************
* File          : DebounceTask.hpp
*
* Description   : 
*
* Project       :
*
* Author        : s.gilbert
*
* Created on    : 20 Feb 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEBOUNCETASK_HPP
#define DEBOUNCETASK_HPP

/******************************************************************************
INCLUDES
*******************************************************************************/

#include "cmsis_os.h"
#include "GPIODebounceTools.hpp"
#include "SubscribeDebounce.hpp"

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

#define DBTASKFREQUENCY_MS    (1)

/*******************************************************************************
TYPES
*******************************************************************************/

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

const extern osThreadDef_t os_thread_def_debounce;

/*******************************************************************************
CONSTANTS
*******************************************************************************/

/*******************************************************************************
NAMESPACE
*******************************************************************************/

class CDebounceTask
{
public:
    CDebounceTask(osThreadDef_t OSThreadDef
                  , uint32_t period);
    ~CDebounceTask(void) = default;
    void start(void);
    void threadFunc(void * ctx);
    bool subscribe(GPIO_TypeDef * GPIO_port
                  , uint16_t GPIO_pin
                  , uint32_t pull_up_down
                  , callbackFuncPtr_t callback
                  , void * ctx);
    void unsubscribe(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin);

private:
    void updateIOs(uint32_t subscribeCount);
    void checkDebouncedIOs(uint32_t subscribeCount);
    bool isAsserted(uint32_t GPIOID);
    static void _ThdFunc(void * pArg);

private:
    osThreadDef_t m_OSThreadDef;
    osThreadId m_threadID;
    uint32_t m_periodMS;
    CGPIOData GPIOData;
    CSubscribeDebounce m_subscribedGPIOs;
};

/*******************************************************************************
INLINE FUNCTION DEFINITIONS
*******************************************************************************/

#endif /* DEBOUNCETASK_HPP */
