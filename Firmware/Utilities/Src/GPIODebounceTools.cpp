/*******************************************************************************
* File          : GPIODebounceTools.hpp
*
* Description   : GPIO debounce tools based on the report and code samples found
*                   here: http://www.ganssle.com/debouncing-pt2.h
*
* Project       :
*
* Author        : Simon Gilbert
*
* Created on    : 19 Feb 2020
*
*******************************************************************************/

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "GPIODebounceTools.hpp"

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
GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
MODULE VARIABLES
*******************************************************************************/

/*******************************************************************************
INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/

/*******************************************************************************
FUNCTION IMPLEMENTATIONS
*******************************************************************************/

/**\brief   Constructor
 *
 * \param   pStateTable - pointer to table memory
 * \param   size        - byte count of table
 *
 * \return  None
 */
CGPIOData::CGPIOData(uint32_t * pStateTable, size_t size)
    : m_stateCB(pStateTable, size / sizeof(uint32_t), false, true)
    , m_pStateTable(pStateTable)
    , m_debounceWindow(size / sizeof(uint32_t))
    , m_state(0)
    , m_changed(0)
{

}

/**\brief   adds new GPIO state to buffer
 *
 * \param   newState    - new value to add to array
 *
 * \return  None
 */
void CGPIOData::update(uint32_t newState)
{
    m_stateCB.write(newState);
    debounce();
}

/**\brief   calculates all de-bounced GPIO states
 *
 * \param   None
 *
 * \return  None
 */
void CGPIOData::debounce(void)
{
    uint32_t debouncedState = 0xffff;
    uint32_t lastDebouncedState = m_state;

    /* calculate debounced states for all IOs, if there is a constant stream of
     * ones or zeros, the io has debounced.
     */
    for(auto i = 0u; i < m_debounceWindow; ++i)
    {
        debouncedState &= m_pStateTable[i];
    }

    /* Calculate what changed. If the IO was high and is now low, XOR'ing 1 and
     * 0 produces a 1. If the IO was low and is now high, XOR'ing 0 and 1
     * also produces a 1. Otherwise, it is 0
     */
    m_changed = debouncedState ^ lastDebouncedState;
    m_state = debouncedState;
}

/**\brief   Calculates if the selected GPIO has changed state and been asserted
 *
 * \param   GPIOID  - GPIO to be checked
 *
 * \return  returns true if the selected debounced GPIO has an edge transition
 */
bool CGPIOData::isAsserted(uint32_t GPIOID)
{
    return ((m_changed & m_state) & GPIOID);
}

/**\brief   Calculates if the selected GPIO has changed state and been deasserted
 *
 * \param   GPIOID  - GPIO to be checked
 *
 * \return  returns true if the selected debounced GPIO has an edge transition
 */
bool CGPIOData::isDeAsserted(uint32_t GPIOID)
{
    return ((m_changed & ~m_state) & GPIOID);
}

/**\brief   Returns the current IO state
 *
 * \param   GPIOID  - GPIO to be checked
 *
 * \return  returns true if the selected GPIO has been asserted
 */
bool CGPIOData::GPIODebouncedState(uint32_t GPIOID)
{
    return (m_state & GPIOID);
}

/**\brief   Has any GPIO changed state
 *
 * \param   None
 *
 * \return  returns 1 in the bit field if any GPIO that has changed state
 */
uint32_t CGPIOData::anyChangedState(void)
{
    return m_changed;
}
