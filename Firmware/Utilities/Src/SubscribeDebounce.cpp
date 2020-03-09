/*******************************************************************************
* File          : SubscribeDebounce.cpp
*
* Description   : 
*
* Project       :
*
* Author        : s.gilbert
*
* Created on    : 2 Mar 2020
*
*******************************************************************************/

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "SubscribeDebounce.hpp"

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
CSubscribeDebounce::CSubscribeDebounce(void * pTable, size_t size)
    : CSubscribeBase(pTable, size)
{}

/**\brief   Configures GPIO for de-bouncing.
 *
 * \param   pSubscription   - pointer to struct containing pin assignment
 *
 * \return  None
 */
void CSubscribeDebounce::configureGPIO(subscription_t * pSubscription)
{
    // Set up GPIO
    HAL_GPIO_DeInit(pSubscription->GPIO_port, pSubscription->GPIO_InitStruct.Pin);
    HAL_GPIO_Init(pSubscription->GPIO_port, &pSubscription->GPIO_InitStruct);
    // Clear any previous triggers
    __HAL_GPIO_EXTI_CLEAR_IT(pSubscription->GPIO_InitStruct.Pin);
}

/**\brief   Removes the pin from the debounce list.
 *
 * \param   pSubscription   - pointer to struct containing pin assignment
 *
 * \return  None
 */
void CSubscribeDebounce::unconfigureGPIO(subscription_t * pSubscription)
{
    HAL_GPIO_DeInit(pSubscription->GPIO_port, pSubscription->GPIO_InitStruct.Pin);
}

