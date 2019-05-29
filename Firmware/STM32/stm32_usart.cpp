
#include "stm32_usart.hpp"
#include "stm32_system.h"

bool STM32_USART_t::init(uint32_t baudrate, STM32_GPIO_t* tx_gpio, STM32_GPIO_t* rx_gpio, STM32_DMAStream_t* tx_dma, STM32_DMAStream_t* rx_dma) {
    if (huart.Instance == USART1)
        __HAL_RCC_USART1_CLK_ENABLE();
    else if (huart.Instance == USART2)
        __HAL_RCC_USART2_CLK_ENABLE();
    else if (huart.Instance == USART3)
        __HAL_RCC_USART3_CLK_ENABLE();
    else if (huart.Instance == UART4)
        __HAL_RCC_UART4_CLK_ENABLE();
    else if (huart.Instance == UART5)
        __HAL_RCC_UART5_CLK_ENABLE();
    else if (huart.Instance == USART6)
        __HAL_RCC_USART6_CLK_ENABLE();
    else
        return false;
    
    tx_dma_ = tx_dma;
    rx_dma_ = rx_dma;

    if (tx_dma) {
        if (!tx_dma->init(tx_dmas, DMA_t::MEMORY, DMA_t::PERIPHERAL, DMA_t::ALIGN_8_BIT, DMA_t::LINEAR, DMA_t::LOW, 1)) {
            return false;
        }
        tx_dma->link(huart, &UART_HandleTypeDef::hdmatx);
    }

    if (rx_dma) {
        if (!rx_dma->init(rx_dmas, DMA_t::PERIPHERAL, DMA_t::MEMORY, DMA_t::ALIGN_8_BIT, DMA_t::CIRCULAR, DMA_t::LOW, 1)) {
            return false;
        }
        rx_dma->link(huart, &UART_HandleTypeDef::hdmarx);
    }

    huart.Init.BaudRate = baudrate;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.StopBits = UART_STOPBITS_1;
    huart.Init.Parity = UART_PARITY_NONE;
    huart.Init.Mode = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart) != HAL_OK)
        return false;

    if (rx_gpio)
        if (!rx_gpio->setup_alternate_function(rx_gpios, gpio_af, GPIO_t::NO_PULL, GPIO_t::VERY_FAST))
            return false;
    if (tx_gpio) // TODO: shouldn't RX be pulled down instead of TX?
        if (!tx_gpio->setup_alternate_function(tx_gpios, gpio_af, GPIO_t::PULL_DOWN, GPIO_t::VERY_FAST))
            return false;

    return true;
}

bool STM32_USART_t::enable_interrupts(uint8_t priority) {
    enable_interrupt(get_irq_number(), priority);

    return (!tx_dma_ || tx_dma_->enable_interrupts(priority))
        && (!rx_dma_ || rx_dma_->enable_interrupts(priority));
}

bool STM32_USART_t::start_rx(uint8_t* data, size_t length) {
    if (!data)
        return false;
    if (HAL_UART_Receive_DMA(&huart, data, length) != HAL_OK)
        return false;

    rx_last_rcv_idx_ = huart.RxXferSize - huart.hdmarx->Instance->NDTR;
    return true;
}

bool STM32_USART_t::stop_rx() {
    return HAL_UART_AbortReceive(&huart) == HAL_OK;
}

bool STM32_USART_t::check_error() {
    return huart.ErrorCode == HAL_UART_ERROR_NONE;
}

bool STM32_USART_t::get_rx_data(uint8_t** data, size_t* length) {
    // Fetch the circular buffer "write pointer", where it would write next
    uint32_t new_rcv_idx = huart.RxXferSize - huart.hdmarx->Instance->NDTR;
    size_t chunk_length;

    if (data) {
        // note that using huart.pRxBuffPtr is only ok in DMA mode
        *data = huart.pRxBuffPtr + rx_last_rcv_idx_;
    }

    if (new_rcv_idx < rx_last_rcv_idx_) {
        chunk_length = huart.RxXferSize - rx_last_rcv_idx_;
        rx_last_rcv_idx_ = 0;
    } else {
        chunk_length = new_rcv_idx - rx_last_rcv_idx_;
        rx_last_rcv_idx_ = new_rcv_idx;
    }

    if (length) {
        *length = chunk_length;
    }
    return chunk_length > 0;
}

bool STM32_USART_t::start_tx(const uint8_t* buf, size_t length, void (*callback)(void*), void* ctx) {
    tx_callback_ = nullptr;
    tx_ctx_ = ctx;
    tx_callback_ = callback;

    // DMA doesn't work with core-coupled memory
    if (reinterpret_cast<uintptr_t>(buf) >= CCMDATARAM_BASE && reinterpret_cast<uintptr_t>(buf) < CCMDATARAM_END) {
        return false;
    }

    // Casting away the const should be fine here since the DMA only reads from it
    if (HAL_UART_Transmit_DMA(&huart, const_cast<uint8_t*>(buf), length) != HAL_OK)
        return false;
    
    return true;
}

bool STM32_USART_t::handle_tx_complete() {
    if (tx_callback_) {
        tx_callback_(tx_ctx_);
    }
    return false;
}

bool STM32_USART_t::handle_error() {
    // TODO: do something meaningful
    return true;
}


/* Interrupt entrypoints -----------------------------------------------------*/

extern "C" {
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void USART6_IRQHandler(void);
}

/** @brief Entrypoint for the USART1 global interrupt. */
void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&usart1.huart);
}

/** @brief Entrypoint for the USART2 global interrupt. */
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&usart2.huart);
}

/** @brief Entrypoint for the USART3 global interrupt. */
void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&usart3.huart);
}

/** @brief Entrypoint for the UART4 global interrupt. */
void UART4_IRQHandler(void) {
    HAL_UART_IRQHandler(&uart4.huart);
}

/** @brief Entrypoint for the UART5 global interrupt. */
void UART5_IRQHandler(void) {
    HAL_UART_IRQHandler(&uart5.huart);
}

/** @brief Entrypoint for the USART6 global interrupt. */
void USART6_IRQHandler(void) {
    HAL_UART_IRQHandler(&usart6.huart);
}


/* HAL callbacks -------------------------------------------------------------*/

static STM32_USART_t* all_usarts[] = { &usart1, &usart2, &usart3, &uart4, &uart5, &usart6 }; // TODO: this prevents compile time garbage collection

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    for (size_t i = 0; i < sizeof(all_usarts) / sizeof(all_usarts[0]); ++i) {
        if (&all_usarts[i]->huart == huart) {
            all_usarts[i]->handle_tx_complete();
        }
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    for (size_t i = 0; i < sizeof(all_usarts) / sizeof(all_usarts[0]); ++i) {
        if (&all_usarts[i]->huart == huart) {
            all_usarts[i]->handle_error();
        }
    }
}


/* Peripheral definitions ----------------------------------------------------*/

const STM32_GPIO_t* usart1_ck_gpios[] = { &pa8, nullptr };
const STM32_GPIO_t* usart1_tx_gpios[] = { &pa9, &pb6, nullptr };
const STM32_GPIO_t* usart1_rx_gpios[] = { &pa10, &pb7, nullptr };
const STM32_GPIO_t* usart1_cts_gpios[] = { &pa11, nullptr };
const STM32_GPIO_t* usart1_rts_gpios[] = { &pa12, nullptr };
const STM32_DMAChannel_t usart1_tx_dmas[] = { {&dma2_stream7, 4}, {0} };
const STM32_DMAChannel_t usart1_rx_dmas[] = { {&dma2_stream2, 4}, {&dma2_stream5, 4}, {0} };

const STM32_GPIO_t* usart2_ck_gpios[] = { &pa4, &pd7, nullptr };
const STM32_GPIO_t* usart2_tx_gpios[] = { &pa2, &pd5, nullptr };
const STM32_GPIO_t* usart2_rx_gpios[] = { &pa3, &pd6, nullptr };
const STM32_GPIO_t* usart2_cts_gpios[] = { &pa0, &pd3, nullptr };
const STM32_GPIO_t* usart2_rts_gpios[] = { &pa1, &pd4, nullptr };
const STM32_DMAChannel_t usart2_tx_dmas[] = { {&dma1_stream6, 4}, {0} };
const STM32_DMAChannel_t usart2_rx_dmas[] = { {&dma1_stream5, 4}, {0} };

const STM32_GPIO_t* usart3_ck_gpios[] = { &pb12, &pc12, &pd10, nullptr };
const STM32_GPIO_t* usart3_tx_gpios[] = { &pb10, &pc10, &pd8, nullptr };
const STM32_GPIO_t* usart3_rx_gpios[] = { &pb11, &pc11, &pd9, nullptr };
const STM32_GPIO_t* usart3_cts_gpios[] = { &pb13, &pd11, nullptr };
const STM32_GPIO_t* usart3_rts_gpios[] = { &pb14, &pd12, nullptr };
const STM32_DMAChannel_t usart3_tx_dmas[] = { {&dma1_stream3, 4}, {&dma1_stream4, 7}, {0} };
const STM32_DMAChannel_t usart3_rx_dmas[] = { {&dma1_stream1, 4}, {0} };

const STM32_GPIO_t* uart4_ck_gpios[] = { nullptr };
const STM32_GPIO_t* uart4_tx_gpios[] = { &pa0, &pc10, nullptr };
const STM32_GPIO_t* uart4_rx_gpios[] = { &pa1, &pc11, nullptr };
const STM32_GPIO_t* uart4_cts_gpios[] = { nullptr };
const STM32_GPIO_t* uart4_rts_gpios[] = { nullptr };
const STM32_DMAChannel_t uart4_tx_dmas[] = { {&dma1_stream4, 4}, {0} };
const STM32_DMAChannel_t uart4_rx_dmas[] = { {&dma1_stream2, 4}, {0} };

const STM32_GPIO_t* uart5_ck_gpios[] = { nullptr };
const STM32_GPIO_t* uart5_tx_gpios[] = { &pc12, nullptr };
const STM32_GPIO_t* uart5_rx_gpios[] = { &pd2, nullptr };
const STM32_GPIO_t* uart5_cts_gpios[] = { nullptr };
const STM32_GPIO_t* uart5_rts_gpios[] = { nullptr };
const STM32_DMAChannel_t uart5_tx_dmas[] = { {&dma1_stream7, 4}, {0} };
const STM32_DMAChannel_t uart5_rx_dmas[] = { {&dma1_stream0, 4}, {0} };

const STM32_GPIO_t* usart6_ck_gpios[] = { &pc8, &pg7, nullptr };
const STM32_GPIO_t* usart6_tx_gpios[] = { &pc6, &pg8, &pg14, nullptr };
const STM32_GPIO_t* usart6_rx_gpios[] = { &pc7, &pg9, nullptr };
const STM32_GPIO_t* usart6_cts_gpios[] = { &pg13, &pg15, nullptr };
const STM32_GPIO_t* usart6_rts_gpios[] = { &pg12, nullptr };
const STM32_DMAChannel_t usart6_tx_dmas[] = { {&dma2_stream6, 5}, {&dma2_stream7, 5}, {0} };
const STM32_DMAChannel_t usart6_rx_dmas[] = { {&dma2_stream1, 5}, {&dma2_stream2, 5}, {0} };

STM32_USART_t usart1(USART1, usart1_tx_gpios, usart1_rx_gpios, GPIO_AF7_USART1, usart1_tx_dmas, usart1_rx_dmas);
STM32_USART_t usart2(USART2, usart2_tx_gpios, usart2_rx_gpios, GPIO_AF7_USART2, usart2_tx_dmas, usart2_rx_dmas);
STM32_USART_t usart3(USART3, usart3_tx_gpios, usart3_rx_gpios, GPIO_AF7_USART3, usart3_tx_dmas, usart3_rx_dmas);
STM32_USART_t uart4(UART4, uart4_tx_gpios, uart4_rx_gpios, GPIO_AF8_UART4, uart4_tx_dmas, uart4_rx_dmas);
STM32_USART_t uart5(UART5, uart5_tx_gpios, uart5_rx_gpios, GPIO_AF8_UART5, uart5_tx_dmas, uart5_rx_dmas);
STM32_USART_t usart6(USART6, usart6_tx_gpios, usart6_rx_gpios, GPIO_AF8_USART6, usart6_tx_dmas, usart6_rx_dmas);

