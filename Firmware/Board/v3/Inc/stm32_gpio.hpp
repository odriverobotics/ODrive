#ifndef __STM32_GPIO_HPP
#define __STM32_GPIO_HPP

#include <gpio.hpp>
#include "stm32f4xx.h"

class STM32_GPIO_t : public GPIO_t {
public:
    GPIO_TypeDef* port;
    uint16_t pin_number;

    STM32_GPIO_t(GPIO_TypeDef* port, uint16_t pin_number) :
            port(port), pin_number(pin_number) {}

    bool init(MODE mode, PULL pull, bool state = false);

    /*
    * @brief Sets up the GPIO for use as an ADC input.
    * Returns true if the setup succeeded and false otherwise.
    * This can be called without calling init() first.
    */
    bool setup_analog();

    /*
    * @brief Sets up the GPIO for use with the specified alternate function.
    * Returns true if the setup succeeded and false otherwise.
    * This can be called without calling init() first.
    * 
    * @param valid_gpios: A nullptr terminated list that specifies all valid
    *        GPIOs for this function. If this GPIO is not part of the list, the
    *        function returns false.
    */
    bool setup_alternate_function(const STM32_GPIO_t** valid_gpios, uint8_t alternate_function, PULL pull, SPEED speed, bool open_drain = false);

    void write(bool value) {
        HAL_GPIO_WritePin(port, 1U << pin_number, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    bool read() {
        return HAL_GPIO_ReadPin(port, 1U << pin_number) != GPIO_PIN_RESET;
    }

    IRQn_Type get_irq_number();
    
    /*
    * @brief Subscribes to an interrupt on the GPIO.
    * Returns true if the setup succeeded and false otherwise.
    */
    bool subscribe(bool rising_edge, bool falling_edge,
        void (*callback)(void*), void* ctx) final;

    /*
    * @brief Removes all callbacks from this pin.
    */
    void unsubscribe();

    /*
    * @brief Checks if this GPIO is in the given NULL-terminated list.
    * Returns true if it is contained and false otherwise. The list must not
    * contain more than 1024 elements.
    */
    bool is_in_list(const STM32_GPIO_t** gpio_list);
    
    void deinit() final;
private:
    bool enable_clock();
};

extern STM32_GPIO_t pa0, pa1, pa2, pa3, pa4, pa5, pa6, pa7;
extern STM32_GPIO_t pa8, pa9, pa10, pa11, pa12, pa13, pa14, pa15;

extern STM32_GPIO_t pb0, pb1, pb2, pb3, pb4, pb5, pb6, pb7;
extern STM32_GPIO_t pb8, pb9, pb10, pb11, pb12, pb13, pb14, pb15;

extern STM32_GPIO_t pc0, pc1, pc2, pc3, pc4, pc5, pc6, pc7;
extern STM32_GPIO_t pc8, pc9, pc10, pc11, pc12, pc13, pc14, pc15;

extern STM32_GPIO_t pd0, pd1, pd2, pd3, pd4, pd5, pd6, pd7;
extern STM32_GPIO_t pd8, pd9, pd10, pd11, pd12, pd13, pd14, pd15;

extern STM32_GPIO_t pe0, pe1, pe2, pe3, pe4, pe5, pe6, pe7;
extern STM32_GPIO_t pe8, pe9, pe10, pe11, pe12, pe13, pe14, pe15;

extern STM32_GPIO_t pf0, pf1, pf2, pf3, pf4, pf5, pf6, pf7;
extern STM32_GPIO_t pf8, pf9, pf10, pf11, pf12, pf13, pf14, pf15;

extern STM32_GPIO_t pg0, pg1, pg2, pg3, pg4, pg5, pg6, pg7;
extern STM32_GPIO_t pg8, pg9, pg10, pg11, pg12, pg13, pg14, pg15;

extern STM32_GPIO_t ph0, ph1, ph2, ph3, ph4, ph5, ph6, ph7;
extern STM32_GPIO_t ph8, ph9, ph10, ph11, ph12, ph13, ph14, ph15;

extern STM32_GPIO_t pi0, pi1, pi2, pi3, pi4, pi5, pi6, pi7;
extern STM32_GPIO_t pi8, pi9, pi10, pi11, pi12, pi13, pi14, pi15;

#endif // __STM32_GPIO_HPP