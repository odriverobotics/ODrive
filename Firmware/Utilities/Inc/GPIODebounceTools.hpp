/*******************************************************************************
* File          : GPIODebounceTools.hpp
*
* Description   : GPIO debounce tools based on the report and code samples found
*                   here: http://www.ganssle.com/debouncing-pt2.htm
*
* Project       :
*
* Author        : Simon Gilbert
*
* Created on    : 19 Feb 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GPIODEBOUNCETOOLS_HPP
#define GPIODEBOUNCETOOLS_HPP

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "CircularBuffer.hpp"

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
NAMESPACE
*******************************************************************************/

class CGPIOData
{
public:
    CGPIOData(uint32_t * pStateTable, size_t size);
    ~CGPIOData(void) = default;
    void update(uint32_t newState);
    void debounce(void);
    bool isAsserted(uint32_t GPIOID);
    bool isDeAsserted(uint32_t GPIOID);
    bool GPIODebouncedState(uint32_t GPIOID);
    uint32_t anyChangedState(void);

private:
    CCBBuffer<uint32_t> m_stateCB;
    uint32_t * m_pStateTable;
    size_t m_debounceWindow;
    uint32_t m_state;
    uint32_t m_changed;
};

/*******************************************************************************
FUNCTION PROTOTYPES
*******************************************************************************/

/*******************************************************************************
INLINE FUNCTIONS
*******************************************************************************/

#endif /* GPIODEBOUNCETOOLS_HPP */
