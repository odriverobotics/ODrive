
#include "stm32_spi.hpp"

#ifdef DMA_REQUEST_SPI1_RX

// H7 series

#define INIT_SPI(spi, rx_dma, tx_dma) do { \
        __HAL_RCC_ ## spi ## _CLK_ENABLE(); \
        rx_dma.Init.Request = DMA_REQUEST_ ## spi ## _RX; \
        tx_dma.Init.Request = DMA_REQUEST_ ## spi ## _TX; \
    } while (0)

#else

// F4 and F7 series

#define DMA_CHANNEL_SPI1  DMA_CHANNEL_3
#define DMA_CHANNEL_SPI2  DMA_CHANNEL_0
#define DMA_CHANNEL_SPI3  DMA_CHANNEL_0

#define INIT_SPI(spi, rx_dma, tx_dma) do { \
        __HAL_RCC_ ## spi ## _CLK_ENABLE(); \
        rx_dma.Init.Channel = DMA_CHANNEL_ ## spi; \
        tx_dma.Init.Channel = DMA_CHANNEL_ ## spi; \
    } while (0)

#endif



Stm32Spi::Stm32Spi(SPI_TypeDef* instance, Stm32DmaStreamRef rx_dma, Stm32DmaStreamRef tx_dma)
    : hspi_{.Instance = instance},
      hdma_rx_{.Instance = rx_dma.get_instance()},
      hdma_tx_{.Instance = tx_dma.get_instance()}
{}

bool Stm32Spi::init() {
    if (hspi_.Instance == SPI1) {
        INIT_SPI(SPI1, hdma_rx_, hdma_tx_);
        freq_ = HAL_RCC_GetPCLK2Freq(); // verified for STM32F405 and STM32F722
    } else if (hspi_.Instance == SPI2) {
        INIT_SPI(SPI2, hdma_rx_, hdma_tx_);
        freq_ = HAL_RCC_GetPCLK1Freq(); // verified for STM32F405 and STM32F722
    } else if (hspi_.Instance == SPI3) {
        INIT_SPI(SPI3, hdma_rx_, hdma_tx_);
        freq_ = HAL_RCC_GetPCLK1Freq(); // verified for STM32F405 and STM32F722
    } else {
        // I'm too lazy to implement this for all instances
        return false;
    }

    __HAL_LINKDMA((&hspi_), hdmatx, hdma_tx_);
    __HAL_LINKDMA((&hspi_), hdmarx, hdma_rx_);

    Stm32DmaStreamRef{hdma_rx_.Instance}.enable_clock();
    Stm32DmaStreamRef{hdma_tx_.Instance}.enable_clock();

    return true;
}

bool Stm32Spi::config(Config config) {
    HAL_DMA_DeInit(&hdma_tx_);
    HAL_DMA_DeInit(&hdma_rx_);
    HAL_SPI_DeInit(&hspi_);

    
    // Prescalers from 2 to 256 are supported
    size_t prescaler_log;
    for (prescaler_log = 1; prescaler_log <= 8; ++prescaler_log) {
        if (config.max_baud_rate * (1 << prescaler_log) >= freq_) {
            break;
        }
    }
    if (prescaler_log >= 9) {
        return false; // requested baud rate too low
    }

    static const uint32_t prescalers[8] = {
        SPI_BAUDRATEPRESCALER_2,
        SPI_BAUDRATEPRESCALER_4,
        SPI_BAUDRATEPRESCALER_8,
        SPI_BAUDRATEPRESCALER_16,
        SPI_BAUDRATEPRESCALER_32,
        SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128,
        SPI_BAUDRATEPRESCALER_256,
    };

    hspi_.Init = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_16BIT,
        .CLKPolarity = config.clk_polarity,
        .CLKPhase = config.clk_phase,
        .NSS = SPI_NSS_SOFT,
        .BaudRatePrescaler = prescalers[prescaler_log - 1],
        .FirstBit = SPI_FIRSTBIT_MSB,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial = 10,
    };

    hdma_tx_.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx_.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx_.Init.MemInc = DMA_MINC_ENABLE;
    if(hspi_.Init.DataSize == SPI_DATASIZE_8BIT) {
        hdma_tx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_tx_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    } else {
        hdma_tx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_tx_.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    }
    hdma_tx_.Init.Mode = DMA_NORMAL;
    hdma_tx_.Init.Priority = DMA_PRIORITY_HIGH; // SPI TX must have higher priority than SPI RX
    hdma_tx_.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_tx_) != HAL_OK) {
        return false;
    }

    hdma_rx_.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx_.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx_.Init.MemInc = DMA_MINC_ENABLE;
    if (hspi_.Init.DataSize == SPI_DATASIZE_8BIT) {
        hdma_rx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_rx_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    } else {
        hdma_rx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_rx_.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    }
    hdma_rx_.Init.Mode = DMA_NORMAL;
    hdma_rx_.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_rx_.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_rx_) != HAL_OK) {
        return false;
    }

    if (HAL_SPI_Init(&hspi_) != HAL_OK) {
        return false;
    }

    // This is necessary to avoid weird behavior when reconfiguring on the fly
    __HAL_SPI_ENABLE(&hspi_);

    return true;
}
