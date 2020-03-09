/*******************************************************************************
* File          : SubscribeBase.cpp
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

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "SubscribeBase.hpp"
#include "stddef.h"

/*******************************************************************************
NAMESPACE
*******************************************************************************/

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

/*******************************************************************************
TYPES
*******************************************************************************/

/*******************************************************************************
CONSTANTS
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

/**\brief   Constructor.
 *
 * \param   pTable  - pointer to the subscription table space
 * \param   size    - byte count of allocated memory
 *
 * \return  returns pointer to active subscription or nullptr if not found
 */
CSubscribeBase::CSubscribeBase(void * pTable, size_t size)
    : subscriptionList(pTable, size)
{
    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);
}

/**\brief   Checks to see if the GPIO details have already got an active
 *          subscription and returns the nodeID.
 *
 * \param   GPIO_port   - pointer to the port struct
 * \param   GPIO_pin    - pin ID
 *
 * \return  returns nodeID to active subscription or (-1)
 */
int32_t CSubscribeBase::findActiveSubscription(GPIO_TypeDef* GPIO_port
                                                            , uint16_t GPIO_pin)
{
    int32_t returnVal = -1;

    for (size_t i = 0; i < subscriptionList.countNodes(); ++i)
    {
        subscription_t subscription;
        subscriptionList.peakFromNode(i, &subscription);
        if ((subscription.GPIO_port == GPIO_port)
                && (subscription.GPIO_InitStruct.Pin == GPIO_pin))
        {
            returnVal = i;
            break;
        }
    }

    return returnVal;
}

/**\brief   Registers a callback to the specified port and pin combination
 *
 * \param   GPIO_port       - pointer to the port struct
 * \param   GPIO_pin        - pin ID
 * \param   pull_up_down    - one of GPIO_NOPULL, GPIO_PULLUP or GPIO_PULLDOWN
 * \param   callback        - pointer to function to use for callback
 * \param   ctx             - pointer to callback argument
 *
 * \return  returns true if successful otherwise returns false
 */
bool CSubscribeBase::subscribe(GPIO_TypeDef * GPIO_port
                           , uint16_t GPIO_pin
                           , uint32_t pull_up_down
                           , callbackFuncPtr_t callback
                           , void * ctx)
{
    bool returnVal = false;
    subscription_t subscription;

    subscription.GPIO_port = GPIO_port;
    subscription.GPIO_InitStruct.Pin = GPIO_pin;
    subscription.GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    subscription.GPIO_InitStruct.Pull = pull_up_down;
    subscription.callback = callback;
    subscription.ctx = ctx;

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
    {
        // check for a pre-configured subscription for the port and pin
        int32_t nodeID = findActiveSubscription(GPIO_port, GPIO_pin);
        // if one exists,  delete it
        if ((-1) != nodeID)
        {
            subscriptionList.deleteNode(nodeID);
        }

        // check for space in the list
        if (subscriptionList.countNodes() < subscriptionList.getMaxNodes())
        {
            if(LL_SUCCESS == subscriptionList.pushToBack(&subscription))
            {
                configureGPIO(&subscription);
                returnVal = true;
            }
        }

        xSemaphoreGive(xSemaphore);
    }

    return returnVal;
}

/**\brief   Unregisters the callback of the specified port and pin combination
 *
 * \param   GPIO_port   - pointer to the port struct
 * \param   GPIO_pin    - pin ID
 *
 * \return  None
 */
void CSubscribeBase::unsubscribe(GPIO_TypeDef * GPIO_port, uint16_t GPIO_pin)
{
    subscription_t subscription;

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
    {
        bool is_pin_in_use = false;

        for (size_t i = 0; i < subscriptionList.countNodes(); ++i)
        {

            subscriptionList.peakFromNode(i, &subscription);
            // if there is a complete match then clear entries
            if ((subscription.GPIO_port == GPIO_port)
                    && (subscription.GPIO_InitStruct.Pin == GPIO_pin))
            {
                subscriptionList.deleteNode(i);
            }
            else
            {
                // if there is only a pin match, we don't want to disable the interrupt
                if (subscription.GPIO_InitStruct.Pin == GPIO_pin)
                {
                    is_pin_in_use = true;
                }
            }
        }

        if (!is_pin_in_use)
        {
            unconfigureGPIO(&subscription);
        }
        xSemaphoreGive(xSemaphore);
    }
}

/**\brief   Gets pointer to subscription list.
 *
 * \param   None
 *
 * \return  None
 */
bool CSubscribeBase::getSubscriptionList(uint32_t tableID, GPIO_TypeDef ** GPIO_port, uint16_t * GPIO_pin)
{
    subscription_t subscription;

    subscriptionList.peakFromNode((tableID -1), &subscription);
    *GPIO_port = subscription.GPIO_port;
    *GPIO_pin = subscription.GPIO_InitStruct.Pin;

    return true;
}

/**\brief   Gets active count on the subscription list.
 *
 * \param   None
 *
 * \return  None
 */
size_t CSubscribeBase::getSubscriptionCount(void)
{
    return subscriptionList.countNodes();
}

/**\brief   Gets pointer to callback function.
 *
 * \param   listID  - subscription entry ID
 *
 * \return  None
 */
callbackFuncPtr_t CSubscribeBase::getCallback(uint32_t listID)
{
    subscription_t * subscription = (subscription_t *)subscriptionList.findNode(listID);

    return subscription->callback;
}
