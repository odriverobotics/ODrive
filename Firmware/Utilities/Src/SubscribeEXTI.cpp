/*******************************************************************************
* File          : SubscribeEXTI.cpp
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

#include "SubscribeEXTI.hpp"

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
CSubscribeEXTI::CSubscribeEXTI(void * pTable, size_t size)
    : CSubscribeBase::CSubscribeBase(pTable, size)
{}

/**\brief   Configures GPIO for external interrupt.
 *
 * \param   pSubscription   - pointer to struct containing pin assignment
 *
 * \return  None
 */
void CSubscribeEXTI::configureGPIO(subscription_t * pSubscription)
{
    // Set up GPIO
    HAL_GPIO_DeInit(pSubscription->GPIO_port, pSubscription->GPIO_InitStruct.Pin);
    HAL_GPIO_Init(pSubscription->GPIO_port, &pSubscription->GPIO_InitStruct);
    // Clear any previous triggers
    __HAL_GPIO_EXTI_CLEAR_IT(pSubscription->GPIO_InitStruct.Pin);
    // Enable interrupt
    HAL_NVIC_SetPriority(getIRQNumber(pSubscription->GPIO_InitStruct.Pin), 0, 0);
    HAL_NVIC_EnableIRQ(getIRQNumber(pSubscription->GPIO_InitStruct.Pin));
}

/**\brief   Disables the external interrupt for the pin.
 *
 * \param   pSubscription   - pointer to struct containing pin assignment
 *
 * \return  None
 */
void CSubscribeEXTI::unconfigureGPIO(subscription_t * pSubscription)
{
    HAL_NVIC_DisableIRQ(getIRQNumber(pSubscription->GPIO_InitStruct.Pin));
}

/**\brief   Returns the IRQ number associated with a certain pin. Note that all
 *          GPIOs with the same pin number map to the same IRQn, no matter which
 *          port they belong to.
 *
 * \param   pin     - pin ID
 *
 * \return  returns IRQ number
 */
IRQn_Type CSubscribeEXTI::getIRQNumber(uint16_t pin)
{
    IRQn_Type returnVal = (IRQn_Type)0;
    uint16_t pin_number = 0;
    uint16_t pinID = (pin >> 1);

    while (pinID)
    {
        pinID >>= 1;
        ++pin_number;
    }

    switch (pin_number)
    {
        case 0: returnVal = EXTI0_IRQn; break;
        case 1: returnVal = EXTI1_IRQn; break;
        case 2: returnVal = EXTI2_IRQn; break;
        case 3: returnVal = EXTI3_IRQn; break;
        case 4: returnVal = EXTI4_IRQn; break;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9: returnVal = EXTI9_5_IRQn; break;
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15: returnVal = EXTI15_10_IRQn; break;
        default: returnVal = (IRQn_Type)0; break; // impossible fixme this 'id' represents the watchdog ISR
    }

    return returnVal;
}

