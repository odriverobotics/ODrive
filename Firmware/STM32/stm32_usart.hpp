#ifndef __STM32_USART_HPP
#define __STM32_USART_HPP

#include <stm32_gpio.hpp>
#include <stm32_dma.hpp>

class STM32_USART_t {
public:
    UART_HandleTypeDef huart;

    const STM32_GPIO_t** tx_gpios;
    const STM32_GPIO_t** rx_gpios;
    uint8_t gpio_af;
    const STM32_DMAChannel_t* tx_dmas;
    const STM32_DMAChannel_t* rx_dmas;

    STM32_DMAStream_t* tx_dma_ = nullptr;
    STM32_DMAStream_t* rx_dma_ = nullptr;
    void (*tx_callback_)(void*) = nullptr;
    void* tx_ctx_ = nullptr;
    size_t rx_last_rcv_idx_ = 0;

    STM32_USART_t(USART_TypeDef* instance, const STM32_GPIO_t** tx_gpios, const STM32_GPIO_t** rx_gpios, uint8_t gpio_af,
            const STM32_DMAChannel_t* tx_dmas, const STM32_DMAChannel_t* rx_dmas) :
        huart{ .Instance = instance },
        tx_gpios(tx_gpios),
        rx_gpios(rx_gpios),
        gpio_af(gpio_af),
        tx_dmas(tx_dmas),
        rx_dmas(rx_dmas) {}

    IRQn_Type get_irq_number() {
        if (huart.Instance == USART1)
            return USART1_IRQn;
        else if (huart.Instance == USART2)
            return USART2_IRQn;
        else if (huart.Instance == USART3)
            return USART3_IRQn;
        else if (huart.Instance == UART4)
            return UART4_IRQn;
        else if (huart.Instance == UART5)
            return UART5_IRQn;
        else if (huart.Instance == USART6)
            return USART6_IRQn;
        else
            return (IRQn_Type)-1;
    }

    bool init(uint32_t baudrate, STM32_GPIO_t* tx_gpio, STM32_GPIO_t* rx_gpio, STM32_DMAStream_t* tx_dma, STM32_DMAStream_t* rx_dma);
    bool enable_interrupts(uint8_t priority);
    bool start_rx(uint8_t* data, size_t length);
    bool stop_rx();
    bool check_error();
    bool get_rx_data(uint8_t** data, size_t* length);
    bool start_tx(const uint8_t* buf, size_t length, void (*callback)(void*), void* ctx);

//private:
    bool handle_tx_complete();
    bool handle_error();
};

extern STM32_USART_t usart1, usart2, usart3, uart4, uart5, usart6;

#endif // __STM32_USART_HPP