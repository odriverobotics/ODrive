#pragma once

#include "gpio.h"
constexpr GPIO_TypeDef* get_gpio_port_by_pin(uint16_t GPIO_pin){
  switch(GPIO_pin){
    case 1: return GPIO_1_GPIO_Port; break;
    case 2: return GPIO_2_GPIO_Port; break;
    case 3: return GPIO_3_GPIO_Port; break;
    case 4: return GPIO_4_GPIO_Port; break;
#ifdef GPIO_5_GPIO_Port
    case 5: return GPIO_5_GPIO_Port; break;
#endif
#ifdef GPIO_6_GPIO_Port
    case 6: return GPIO_6_GPIO_Port; break;
#endif
#ifdef GPIO_7_GPIO_Port
    case 7: return GPIO_7_GPIO_Port; break;
#endif
#ifdef GPIO_8_GPIO_Port
    case 8: return GPIO_8_GPIO_Port; break;
#endif
    default: return GPIO_1_GPIO_Port;
  }
}

constexpr uint16_t get_gpio_pin_by_pin(uint16_t GPIO_pin){
  switch(GPIO_pin){
    case 1: return GPIO_1_Pin; break;
    case 2: return GPIO_2_Pin; break;
    case 3: return GPIO_3_Pin; break;
    case 4: return GPIO_4_Pin; break;
#ifdef GPIO_5_Pin
    case 5: return GPIO_5_Pin; break;
#endif
#ifdef GPIO_6_Pin
    case 6: return GPIO_6_Pin; break;
#endif
#ifdef GPIO_7_Pin
    case 7: return GPIO_7_Pin; break;
#endif
#ifdef GPIO_8_Pin
    case 8: return GPIO_8_Pin; break;
#endif
    default: return GPIO_1_Pin;
  }
}
