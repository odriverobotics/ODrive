/*******************************************************************************
* File          : SubscribeBase.hpp
*
* Description   : 
*
* Project       :
*
* Author        : s.gilbert
*
* Created on    : 19 Feb 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SUBSCRIPTIONTOOLS_HPP
#define SUBSCRIPTIONTOOLS_HPP

/******************************************************************************
INCLUDES
*******************************************************************************/

#include "LinkedList.hpp"
#include "stm32f4xx_hal.h"
#include "freeRTOS.h"
#include "semphr.h"

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

/*******************************************************************************
TYPES
*******************************************************************************/

typedef void (* callbackFuncPtr_t)(void *);

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
CONSTANTS
*******************************************************************************/

/*******************************************************************************
NAMESPACE
*******************************************************************************/

/*  Base subscription class, hosts the subscription table and basic tools for
 *  registering, finding, and deregistering callbacks to pins
 */
class CSubscribeBase
{
public:
    typedef struct
    {
      GPIO_TypeDef * GPIO_port;
      GPIO_InitTypeDef GPIO_InitStruct;
      callbackFuncPtr_t callback;
      void * ctx;
    } subscription_t;

public:
    CSubscribeBase(void * m_pSubscriptions, size_t size);
    virtual ~CSubscribeBase(void) = default;
    bool subscribe(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin,
                   uint32_t pull_up_down,
                   callbackFuncPtr_t callback, void * ctx);
    void unsubscribe(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin);
    bool getSubscriptionList(uint32_t tableID, GPIO_TypeDef ** GPIO_port, uint16_t * GPIO_pin);
    size_t getSubscriptionCount(void);
    callbackFuncPtr_t getCallback(uint32_t listID);

private:
    int32_t findActiveSubscription(GPIO_TypeDef* GPIO_port
                                            , uint16_t GPIO_pin);
    virtual void configureGPIO(subscription_t * pSubscription) = 0;
    virtual void unconfigureGPIO(subscription_t * pSubscription) = 0;

private:
    CLinkedList<subscription_t> subscriptionList;
//    StaticSemaphore_t xSemaphoreBuffer;
    SemaphoreHandle_t xSemaphore;
};

/*******************************************************************************
INLINE FUNCTION DEFINITIONS
*******************************************************************************/

#endif /* SUBSCRIPTIONTOOLS_HPP */
