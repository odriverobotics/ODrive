#ifndef __STM32_GPIO_HPP
#define __STM32_GPIO_HPP

#include "stm32_system.h"

class Stm32Gpio {
public:
    static const Stm32Gpio none;

    Stm32Gpio() : port_(nullptr), pin_mask_(0) {}
    constexpr Stm32Gpio(GPIO_TypeDef* port, uint16_t pin) : port_(port), pin_mask_(pin) {}

    operator bool() const { return port_ && pin_mask_; }
    bool operator==(Stm32Gpio other) const { return port_ == other.port_ && pin_mask_ == other.pin_mask_; }

    /**
     * @brief Configures the GPIO with the specified parameters.
     * 
     * This can be done regardless of the current state of the GPIO.
     * 
     * If any subscription is in place, it is not disabled by this function.
     */
    bool config(uint32_t mode, uint32_t pull,
        uint32_t speed = GPIO_SPEED_FREQ_LOW, uint32_t alternate_function = 0);

    void write(bool state) {
        if (port_) {
            HAL_GPIO_WritePin(port_, pin_mask_, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }

    bool read() const {
        return port_ && (port_->IDR & pin_mask_);
    }

    /**
     * @brief Subscribes to external interrupts on the specified GPIO.
     * 
     * Before calling this function the gpio should most likely be configured as
     * input (however this is not mandatory, the interrupt works in output mode
     * too).
     * Also you need to enable the EXTIx_IRQn interrupt vectors in the NVIC,
     * otherwise the subscription won't have any effect.
     * 
     * Only one subscription is allowed per pin number. I.e. it is not possible
     * to set up a subscription for both PA0 and PB0 at the same time.
     * 
     * This function is thread-safe with respect to all other public functions
     * of this class.
     * 
     * Returns true if the subscription was set up successfully or false otherwise.
     */
    bool subscribe(bool rising_edge, bool falling_edge, void (*callback)(void*), void* ctx);

    /**
     * @brief Unsubscribes from external interrupt on the specified GPIO.
     * 
     * If no subscription was active for this GPIO, calling this function has no
     * effect.
     *
     * This function is thread-safe with respect to all other public functions
     * of this class, however it must not be called from an interrupt routine
     * running at a higher priority than the interrupt that is being unsubscribed.
     *
     * After this function returns the callback given to subscribe() will no
     * longer be invoked.
     */
    void unsubscribe();

    uint16_t get_pin_number() {
        uint16_t pin_number = 0;
        uint16_t pin_mask = pin_mask_ >> 1;
        while (pin_mask) {
            pin_mask >>= 1;
            pin_number++;
        }
        return pin_number;
    }

    bool enable_clock() {
        switch ((uint32_t)port_) {
            case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
            case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
            case GPIOC_BASE: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
            case GPIOD_BASE: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
            case GPIOE_BASE: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
            case GPIOF_BASE: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
            case GPIOG_BASE: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
            case GPIOH_BASE: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
            default: return false;
        }
        return true;
    }

    GPIO_TypeDef* port_;
    uint16_t pin_mask_; // TODO: store pin_number_ instead of pin_mask_
};

#endif // __STM32_GPIO_HPP
