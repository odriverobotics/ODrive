/*******************************************************************************
* File          : DebounceTask.cpp
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

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "DebounceTask.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f405xx.h"
#include "stdint.h"
#include "TaskConfigs.hpp"

/*******************************************************************************
NAMESPACE
*******************************************************************************/

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

#define DEBOUNCE_TIME_MS (10)

/*******************************************************************************
TYPES
*******************************************************************************/

/*******************************************************************************
CONSTANTS
*******************************************************************************/

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

const osThreadDef_t os_thread_def_debounce = {
        DebounceTask.pThreadName,
        nullptr,    // this gets populated in the task.start routine
        DebounceTask.priority,
        0,
        DebounceTask.stackSize
};

/*******************************************************************************
MODULE VARIABLES
*******************************************************************************/

uint8_t subscriptionTable[(sizeof(CNode<CSubscribeDebounce::subscription_t>) + sizeof(bool))]; // memory space for subscription table, effective number of active GPIO subscriptions at a given time.
uint32_t debounceTable[DEBOUNCE_TIME_MS * DBTASKFREQUENCY_MS];                  //  debounce stabilisiation period x task frequency

/*******************************************************************************
MODULE FUNCTION DECLARATIONS
*******************************************************************************/

/*******************************************************************************
FUNCTION DEFINITIONS
*******************************************************************************/

/**\brief   Constructor.
 *
 * \param   OSThreadDef - thread config struct
 * \param   period      - task period in ms
 *
 * \return  None
 */
CDebounceTask::CDebounceTask(osThreadDef_t OSThreadDef, uint32_t period)
    : m_OSThreadDef(OSThreadDef)
    , m_threadID(nullptr)
    , m_periodMS(period)
    , GPIOData(debounceTable, sizeof(debounceTable))
    , m_subscribedGPIOs(subscriptionTable, sizeof(subscriptionTable))
{}

/**\brief   Registers a callback to the specified port and pin combination.
 *
 * \param   GPIO_port       - pointer to the port struct
 * \param   GPIO_pin        - pin ID
 * \param   pull_up_down    - one of GPIO_NOPULL, GPIO_PULLUP or GPIO_PULLDOWN
 * \param   callback        - pointer to function to use for callback
 * \param   ctx             - pointer to callback argument
 *
 * \return  returns true if successful otherwise returns false
 */
bool CDebounceTask::subscribe(GPIO_TypeDef * GPIO_port
                              , uint16_t GPIO_pin
                              , uint32_t pull_up_down
                              , callbackFuncPtr_t callback
                              , void * ctx)
{
    return m_subscribedGPIOs.subscribe(GPIO_port
                                , GPIO_pin
                                , pull_up_down
                                , callback
                                , ctx);
}

/**\brief   Unregisters the callback of the specified port and pin combination
 *
 * \param   GPIO_port   - pointer to the port struct
 * \param   GPIO_pin    - pin ID
 *
 * \return  None
 */
void CDebounceTask::unsubscribe(GPIO_TypeDef * GPIO_port, uint16_t GPIO_pin)
{
    m_subscribedGPIOs.unsubscribe(GPIO_port , GPIO_pin);
}

/**\brief   main thread function.
 *
 * \param   ctx - argument passed in from scheduler
 *
 * \return  None
 */
void CDebounceTask::threadFunc(void * ctx)
{
    (void)ctx;

    for(;;)
    {
        auto subscribeCount = m_subscribedGPIOs.getSubscriptionCount();
        updateIOs(subscribeCount);
        checkDebouncedIOs(subscribeCount);
        osDelay(m_periodMS);
    }
}

/**\brief   Gets the current IO states of all registered IOs and pushes values
 *          to the debounce arrays. Then updates debounce states.
 *
 * \param   subscribeCount  - number of entries in list
 *
 * \return  None
 */
void CDebounceTask::updateIOs(uint32_t subscribeCount)
{
    GPIO_TypeDef * GPIO_port = nullptr;
    uint16_t pGPIO_pin = 0;
    uint32_t pinList = 0;

    for(auto i = 0u; i < subscribeCount; ++i)
    {
        m_subscribedGPIOs.getSubscriptionList(i, &GPIO_port, &pGPIO_pin);
        pinList |= (HAL_GPIO_ReadPin(GPIO_port, pGPIO_pin) << i);
    }

    GPIOData.update(pinList);
}

/**\brief   Checks the debounced IOs for an assert and calls the associated
 *          callback.
 *
 * \param   subscribeCount  - number of entries in list
 *
 * \return  None
 */
void CDebounceTask::checkDebouncedIOs(uint32_t subscribeCount)
{
    for(auto i = 0u; i < subscribeCount; ++i)
    {
        /* isAsserted() is true only when the IO (after debouncing) initially
         * changes state. Will not be true again until the IO is first
         * de-asserted.
         */
        if(GPIOData.isAsserted((i + 1)))
        {
            callbackFuncPtr_t callback = m_subscribedGPIOs.getCallback(i);
            if(callback)
            {
                callback(nullptr);
            }
        }
    }
}

/**\brief   Has selected GPIO been asserted.
 *
 * \param   GPIOID  - GPIO to be checked
 *
 * \return  returns true if the selected GPIO has been pressed
 */
bool CDebounceTask::isAsserted(uint32_t GPIOID)
{
    return GPIOData.isAsserted(GPIOID);
}

/**\brief   main thread function. Allows call of Virtual main function of child
 *          function.
 *
 * \param   pArg    - pointer to the created class, passed in from scheduler
 *
 * \return  None
 */
void CDebounceTask::_ThdFunc(void * pArg)
{
    if(nullptr != pArg)
    {
        ((CDebounceTask*)pArg)->threadFunc(nullptr);
    }
}

/**\brief   starts the debounce task and associated peripherals.
 *
 * \param   None
 *
 * \return  None
 */
void CDebounceTask::start(void)
{
    m_OSThreadDef.pthread = (void (*)(void *))&this->_ThdFunc;
    m_threadID = osThreadCreate(&m_OSThreadDef, this);
}
