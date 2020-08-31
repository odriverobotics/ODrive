#ifndef __COMPONENT_HPP
#define __COMPONENT_HPP

#include <stdint.h>

class ComponentBase {
public:
    /**
     * @brief Shall run the update action of this component.
     * 
     * This function gets called in a low priority interrupt context and is
     * allowed to call CMSIS functions.
     * 
     * @param timestamp: The timestamp (in HCLK ticks) for which this update
     * is run.
     */
    virtual void update(uint32_t timestamp) = 0;
};

#endif // __COMPONENT_HPP