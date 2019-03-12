
#include "stm32_gpio.hpp"

bool STM32_GPIO_t::enable_clock() {
    if (port == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF)
        __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG)
        __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (port == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();
    else if (port == GPIOI)
        __HAL_RCC_GPIOI_CLK_ENABLE();
    else
        return false;
    return true;    
}

bool STM32_GPIO_t::init(MODE mode, PULL pull, bool state) {
    if (!enable_clock()) {
        return false;
    }

    if (mode == OUTPUT) {
        write(state);
    }

    // TODO: either check if in use or deinit (unsubscribe interrupt callbacks)

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = 1U << pin_number;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    if (mode == INPUT)
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    else if (mode == OUTPUT)
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    else if (mode == OPEN_DRAIN)
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    else
        return false;

    if (pull == PULL_UP)
        GPIO_InitStruct.Pull = GPIO_PULLUP;
    else if (pull == PULL_DOWN)
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    else if (pull == NO_PULL)
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    else
        return false;

    HAL_GPIO_Init(port, &GPIO_InitStruct);

    return true;
}

bool STM32_GPIO_t::setup_analog() {
    // clock doesn't need to be enabled (?)

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = 1U << pin_number;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
    return true;
}

bool STM32_GPIO_t::setup_alternate_function(const STM32_GPIO_t** valid_gpios, uint8_t alternate_function, PULL pull, SPEED speed, bool open_drain) {
    if (!enable_clock()) {
        return false;
    }

    if (valid_gpios && !is_in_list(valid_gpios))
        return false;

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = 1U << pin_number;
    GPIO_InitStruct.Mode = open_drain ? GPIO_MODE_AF_OD : GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = alternate_function;

    if (pull == PULL_UP)
        GPIO_InitStruct.Pull = GPIO_PULLUP;
    else if (pull == PULL_DOWN)
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    else if (pull == NO_PULL)
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    else
        return false;

    if (speed == SLOW)
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    else if (speed == MEDIUM)
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    else if (speed == FAST)
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    else if (speed == VERY_FAST)
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    else
        return false;
        
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    return true;
}

// Expected subscriptions: 2x step signal + 2x encoder index signal
#define MAX_SUBSCRIPTIONS 10
struct subscription_t {
    STM32_GPIO_t* gpio;
    void (*callback)(void*);
    void* ctx;
} subscriptions[MAX_SUBSCRIPTIONS] = { 0 };
size_t n_subscriptions = 0;


bool STM32_GPIO_t::subscribe(bool rising_edge, bool falling_edge,
        void (*callback)(void*), void* ctx) {
  
    // Register handler (or reuse existing registration)
    // TODO: make thread safe
    struct subscription_t* subscription = NULL;
    for (size_t i = 0; i < n_subscriptions; ++i) {
        if (subscriptions[i].gpio->port == this->port &&
            subscriptions[i].gpio->pin_number == this->pin_number)
            subscription = &subscriptions[i];
    }
    if (!subscription) {
        if (n_subscriptions >= MAX_SUBSCRIPTIONS)
            return false;
        subscription = &subscriptions[n_subscriptions++];
    }

    *subscription = (struct subscription_t){
        .gpio = this,
        .callback = callback,
        .ctx = ctx
    };

    // Set up GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = 1U << pin_number;
    if (rising_edge && falling_edge) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    } else if (rising_edge) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    } else if (falling_edge) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    } else {
        return false;
    }


    GPIO_InitStruct.Pull = (port->PUPDR >> (pin_number * 2U)) & GPIO_PUPDR_PUPDR0;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // Enable interrupt
    HAL_NVIC_SetPriority(get_irq_number(), 0, 0);
    HAL_NVIC_EnableIRQ(get_irq_number());
    return true;
}

void STM32_GPIO_t::unsubscribe() {
    bool irq_number_in_use = false;
    for (size_t i = 0; i < n_subscriptions; ++i) {
        if (subscriptions[i].gpio->port == this->port &&
            subscriptions[i].gpio->pin_number == this->pin_number) {
            subscriptions[i].gpio = nullptr;
            subscriptions[i].callback = nullptr;
            subscriptions[i].ctx = nullptr;
        } else if (subscriptions[i].gpio->get_irq_number() == get_irq_number()) {
            irq_number_in_use = true;
        }
    }
    if (!irq_number_in_use)
        HAL_NVIC_DisableIRQ(get_irq_number());
}

/*
* @brief Returns the IRQ number associated with a certain pin.
* Note that all GPIOs with the same pin number map to the same IRQn,
* no matter which port they belong to.
*/
IRQn_Type STM32_GPIO_t::get_irq_number() {
    switch (pin_number) {
        case 0: return EXTI0_IRQn;
        case 1: return EXTI1_IRQn;
        case 2: return EXTI2_IRQn;
        case 3: return EXTI3_IRQn;
        case 4: return EXTI4_IRQn;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9: return EXTI9_5_IRQn;
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15: return EXTI15_10_IRQn;
        default: return (IRQn_Type)-1; // impossible
    }
}

bool STM32_GPIO_t::is_in_list(const STM32_GPIO_t** gpio_list) {
    // It is unlikely that we check against more than 1024 pins but the upper
    // bound prevents a potential endless loop.
    for (size_t i = 0; i < 1024; ++i) {
        if (gpio_list[i] == this)
            return true;
        if (!gpio_list[i])
            return false;
    }
    return false;
}

void STM32_GPIO_t::deinit() {
    unsubscribe();
    HAL_GPIO_DeInit(port, 1U << pin_number);
}


/* Interrupt entrypoints -----------------------------------------------------*/

extern "C" {
void EXTI0_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t pin_mask);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin_mask) {
    // Dispatch processing of external interrupts based on source
    for (size_t i = 0; i < n_subscriptions; ++i) {
        if ((1U << subscriptions[i].gpio->pin_number) == pin_mask) // TODO: check for port
            if (subscriptions[i].callback)
                subscriptions[i].callback(subscriptions[i].ctx);
    }
}

/** @brief Entrypoint for the EXTI line0 interrupt. */
void EXTI0_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/** @brief Entrypoint for the EXTI line2 interrupt. */
void EXTI2_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/** @brief Entrypoint for the EXTI line3 interrupt. */
void EXTI3_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

/** @brief Entrypoint for the EXTI line4 interrupt. */
void EXTI4_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/** @brief Entrypoint for the EXTI lines 5-9 interrupt. */
void EXTI9_5_IRQHandler(void) {
  // The true source of the interrupt is checked inside HAL_GPIO_EXTI_IRQHandler() 
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

/** @brief This function handles EXTI lines 10-15 interrupt. */
void EXTI15_10_IRQHandler(void) {
  // The true source of the interrupt is checked inside HAL_GPIO_EXTI_IRQHandler() 
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}


/* Peripheral definitions ----------------------------------------------------*/

STM32_GPIO_t pa0(GPIOA, 0), pa1(GPIOA, 1), pa2(GPIOA, 2), pa3(GPIOA, 3);
STM32_GPIO_t pa4(GPIOA, 4), pa5(GPIOA, 5), pa6(GPIOA, 6), pa7(GPIOA, 7);
STM32_GPIO_t pa8(GPIOA, 8), pa9(GPIOA, 9), pa10(GPIOA, 10), pa11(GPIOA, 11);
STM32_GPIO_t pa12(GPIOA, 12), pa13(GPIOA, 13), pa14(GPIOA, 14), pa15(GPIOA, 15);

STM32_GPIO_t pb0(GPIOB, 0), pb1(GPIOB, 1), pb2(GPIOB, 2), pb3(GPIOB, 3);
STM32_GPIO_t pb4(GPIOB, 4), pb5(GPIOB, 5), pb6(GPIOB, 6), pb7(GPIOB, 7);
STM32_GPIO_t pb8(GPIOB, 8), pb9(GPIOB, 9), pb10(GPIOB, 10), pb11(GPIOB, 11);
STM32_GPIO_t pb12(GPIOB, 12), pb13(GPIOB, 13), pb14(GPIOB, 14), pb15(GPIOB, 15);

STM32_GPIO_t pc0(GPIOC, 0), pc1(GPIOC, 1), pc2(GPIOC, 2), pc3(GPIOC, 3);
STM32_GPIO_t pc4(GPIOC, 4), pc5(GPIOC, 5), pc6(GPIOC, 6), pc7(GPIOC, 7);
STM32_GPIO_t pc8(GPIOC, 8), pc9(GPIOC, 9), pc10(GPIOC, 10), pc11(GPIOC, 11);
STM32_GPIO_t pc12(GPIOC, 12), pc13(GPIOC, 13), pc14(GPIOC, 14), pc15(GPIOC, 15);

STM32_GPIO_t pd0(GPIOD, 0), pd1(GPIOD, 1), pd2(GPIOD, 2), pd3(GPIOD, 3);
STM32_GPIO_t pd4(GPIOD, 4), pd5(GPIOD, 5), pd6(GPIOD, 6), pd7(GPIOD, 7);
STM32_GPIO_t pd8(GPIOD, 8), pd9(GPIOD, 9), pd10(GPIOD, 10), pd11(GPIOD, 11);
STM32_GPIO_t pd12(GPIOD, 12), pd13(GPIOD, 13), pd14(GPIOD, 14), pd15(GPIOD, 15);

STM32_GPIO_t pe0(GPIOE, 0), pe1(GPIOE, 1), pe2(GPIOE, 2), pe3(GPIOE, 3);
STM32_GPIO_t pe4(GPIOE, 4), pe5(GPIOE, 5), pe6(GPIOE, 6), pe7(GPIOE, 7);
STM32_GPIO_t pe8(GPIOE, 8), pe9(GPIOE, 9), pe10(GPIOE, 10), pe11(GPIOE, 11);
STM32_GPIO_t pe12(GPIOE, 12), pe13(GPIOE, 13), pe14(GPIOE, 14), pe15(GPIOE, 15);

STM32_GPIO_t pf0(GPIOF, 0), pf1(GPIOF, 1), pf2(GPIOF, 2), pf3(GPIOF, 3);
STM32_GPIO_t pf4(GPIOF, 4), pf5(GPIOF, 5), pf6(GPIOF, 6), pf7(GPIOF, 7);
STM32_GPIO_t pf8(GPIOF, 8), pf9(GPIOF, 9), pf10(GPIOF, 10), pf11(GPIOF, 11);
STM32_GPIO_t pf12(GPIOF, 12), pf13(GPIOF, 13), pf14(GPIOF, 14), pf15(GPIOF, 15);

STM32_GPIO_t pg0(GPIOG, 0), pg1(GPIOG, 1), pg2(GPIOG, 2), pg3(GPIOG, 3);
STM32_GPIO_t pg4(GPIOG, 4), pg5(GPIOG, 5), pg6(GPIOG, 6), pg7(GPIOG, 7);
STM32_GPIO_t pg8(GPIOG, 8), pg9(GPIOG, 9), pg10(GPIOG, 10), pg11(GPIOG, 11);
STM32_GPIO_t pg12(GPIOG, 12), pg13(GPIOG, 13), pg14(GPIOG, 14), pg15(GPIOG, 15);

STM32_GPIO_t ph0(GPIOH, 0), ph1(GPIOH, 1), ph2(GPIOH, 2), ph3(GPIOH, 3);
STM32_GPIO_t ph4(GPIOH, 4), ph5(GPIOH, 5), ph6(GPIOH, 6), ph7(GPIOH, 7);
STM32_GPIO_t ph8(GPIOH, 8), ph9(GPIOH, 9), ph10(GPIOH, 10), ph11(GPIOH, 11);
STM32_GPIO_t ph12(GPIOH, 12), ph13(GPIOH, 13), ph14(GPIOH, 14), ph15(GPIOH, 15);

STM32_GPIO_t pi0(GPIOI, 0), pi1(GPIOI, 1), pi2(GPIOI, 2), pi3(GPIOI, 3);
STM32_GPIO_t pi4(GPIOI, 4), pi5(GPIOI, 5), pi6(GPIOI, 6), pi7(GPIOI, 7);
STM32_GPIO_t pi8(GPIOI, 8), pi9(GPIOI, 9), pi10(GPIOI, 10), pi11(GPIOI, 11);
STM32_GPIO_t pi12(GPIOI, 12), pi13(GPIOI, 13), pi14(GPIOI, 14), pi15(GPIOI, 15);
