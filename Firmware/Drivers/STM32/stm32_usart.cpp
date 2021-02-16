
#include "stm32_usart.hpp"

#ifdef DMA_REQUEST_USART2_RX

// H7 series

#define INIT_UART(uart, rx_dma, tx_dma) do { \
        __HAL_RCC_ ## uart ## _CLK_ENABLE(); \
        rx_dma.Init.Request = DMA_REQUEST_ ## uart ## _RX; \
        tx_dma.Init.Request = DMA_REQUEST_ ## uart ## _TX; \
    } while (0)

#else

// F4 and F7 series

#define DMA_CHANNEL_USART1  DMA_CHANNEL_4
#define DMA_CHANNEL_USART2  DMA_CHANNEL_4
#define DMA_CHANNEL_USART3  DMA_CHANNEL_4
#define DMA_CHANNEL_UART4   DMA_CHANNEL_4
#define DMA_CHANNEL_UART5   DMA_CHANNEL_4
#define DMA_CHANNEL_USART6  DMA_CHANNEL_5

#define INIT_UART(uart, rx_dma, tx_dma) do { \
        __HAL_RCC_ ## uart ## _CLK_ENABLE(); \
        rx_dma.Init.Channel = DMA_CHANNEL_ ## uart; \
        tx_dma.Init.Channel = DMA_CHANNEL_ ## uart; \
    } while (0)

#endif



Stm32Usart::Stm32Usart(USART_TypeDef* instance, Stm32DmaStreamRef rx_dma, Stm32DmaStreamRef tx_dma)
    : huart_{.Instance = instance},
      hdma_rx_{.Instance = rx_dma.get_instance()},
      hdma_tx_{.Instance = tx_dma.get_instance()}
{}

bool Stm32Usart::init(uint32_t baudrate) {
    if (huart_.Instance == USART1) {
        INIT_UART(USART1, hdma_rx_, hdma_tx_);
    } else if (huart_.Instance == USART2) {
        INIT_UART(USART2, hdma_rx_, hdma_tx_);
    } else if (huart_.Instance == UART4) {
        INIT_UART(UART4, hdma_rx_, hdma_tx_);
    } else {
        // I'm too lazy to implement this for all instances
        return false;
    }

    Stm32DmaStreamRef{(DMA_Stream_TypeDef*)hdma_rx_.Instance}.enable_clock();
    Stm32DmaStreamRef{(DMA_Stream_TypeDef*)hdma_rx_.Instance}.enable_clock();

    hdma_rx_.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx_.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx_.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx_.Init.Mode = DMA_CIRCULAR;
    hdma_rx_.Init.Priority = DMA_PRIORITY_LOW;
    hdma_rx_.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_rx_) != HAL_OK) {
        return false;
    }

    __HAL_LINKDMA((&huart_), hdmarx, hdma_rx_);

    hdma_tx_.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx_.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx_.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx_.Init.Mode = DMA_NORMAL;
    hdma_tx_.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tx_.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tx_) != HAL_OK) {
        return false;
    }

    __HAL_LINKDMA((&huart_), hdmatx, hdma_tx_);

    huart_.Init.BaudRate = baudrate;
    huart_.Init.WordLength = UART_WORDLENGTH_8B;
    huart_.Init.StopBits = UART_STOPBITS_1;
    huart_.Init.Parity = UART_PARITY_NONE;
    huart_.Init.Mode = UART_MODE_TX_RX;
    huart_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart_) != HAL_OK) {
        return false;
    }

#if defined(IS_UART_FIFO_INSTANCE)
    if (IS_UART_FIFO_INSTANCE(huart_.Instance)) {
        if (HAL_UARTEx_SetTxFifoThreshold(&huart_, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
            return false;
        }
        if (HAL_UARTEx_SetRxFifoThreshold(&huart_, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
            return false;
        }
        if (HAL_UARTEx_DisableFifoMode(&huart_) != HAL_OK) {
            return false;
        }
    }
#endif

    return true;
}
