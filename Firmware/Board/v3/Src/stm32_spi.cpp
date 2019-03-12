
#include "stm32_spi.hpp"

bool STM32_SPI_t::init(STM32_GPIO_t* sck_gpio, STM32_GPIO_t* miso_gpio, STM32_GPIO_t* mosi_gpio, STM32_DMAStream_t* tx_dma, STM32_DMAStream_t* rx_dma) {
    if (hspi.Instance == SPI1)
        __HAL_RCC_SPI1_CLK_ENABLE();
    else if (hspi.Instance == SPI2)
        __HAL_RCC_SPI2_CLK_ENABLE();
    else if (hspi.Instance == SPI3)
        __HAL_RCC_SPI3_CLK_ENABLE();
    else
        return false;
    
    if (tx_dma) {
        if (!tx_dma->init(tx_dmas, DMA_t::MEMORY, DMA_t::PERIPHERAL, DMA_t::ALIGN_16_BIT, DMA_t::LINEAR, DMA_t::MEDIUM)) {
            return false;
        }
        tx_dma->link(hspi, &SPI_HandleTypeDef::hdmatx);
    }

    if (rx_dma) {
        if (!rx_dma->init(rx_dmas, DMA_t::PERIPHERAL, DMA_t::MEMORY, DMA_t::ALIGN_16_BIT, DMA_t::LINEAR, DMA_t::MEDIUM)) {
            return false;
        }
        rx_dma->link(hspi, &SPI_HandleTypeDef::hdmarx);
    }

    hspi.Init.Mode = SPI_MODE_MASTER;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.NSS = SPI_NSS_SOFT;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi) != HAL_OK)
        return false;

    if (sck_gpio)
        if (!sck_gpio->setup_alternate_function(sck_gpios, gpio_af, GPIO_t::NO_PULL, GPIO_t::VERY_FAST))
            return false;
    if (miso_gpio)
        if (!miso_gpio->setup_alternate_function(miso_gpios, gpio_af, GPIO_t::NO_PULL, GPIO_t::VERY_FAST))
            return false;
    if (mosi_gpio)
        if (!mosi_gpio->setup_alternate_function(mosi_gpios, gpio_af, GPIO_t::NO_PULL, GPIO_t::VERY_FAST))
            return false;

    HAL_NVIC_SetPriority(get_irq_number(), 5, 0);
    HAL_NVIC_EnableIRQ(get_irq_number());

    return true;
}


/* Interrupt entrypoints -----------------------------------------------------*/

extern "C" {
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void SPI3_IRQHandler(void);
}

/** @brief Entrypoint for the SPI1 global interrupt. */
void SPI1_IRQHandler(void) {
  HAL_SPI_IRQHandler(&spi1.hspi);
}

/** @brief Entrypoint for the SPI2 global interrupt. */
void SPI2_IRQHandler(void) {
  HAL_SPI_IRQHandler(&spi2.hspi);
}

/** @brief Entrypoint for the SPI3 global interrupt. */
void SPI3_IRQHandler(void) {
  HAL_SPI_IRQHandler(&spi3.hspi);
}


/* Peripheral definitions ----------------------------------------------------*/

const STM32_GPIO_t* spi1_nss_gpios[] = { &pa4, &pa15, nullptr };
const STM32_GPIO_t* spi1_sck_gpios[] = { &pa5, &pb3, nullptr };
const STM32_GPIO_t* spi1_miso_gpios[] = { &pa6, &pb4, nullptr };
const STM32_GPIO_t* spi1_mosi_gpios[] = { &pa7, &pb5, nullptr };
const STM32_DMAChannel_t spi1_tx_dmas[] = { {&dma2_stream3, 3}, {&dma2_stream5, 3}, {0} };
const STM32_DMAChannel_t spi1_rx_dmas[] = { {&dma2_stream0, 3}, {&dma2_stream2, 3}, {0} };

const STM32_GPIO_t* spi2_nss_gpios[] = { &pb9, &pb12, &pi0, nullptr };
const STM32_GPIO_t* spi2_sck_gpios[] = { &pb10, &pb13, &pi1, nullptr };
const STM32_GPIO_t* spi2_miso_gpios[] = { &pb14, &pc2, &pi2, nullptr };
const STM32_GPIO_t* spi2_mosi_gpios[] = { &pb15, &pc3, &pi3, nullptr };
const STM32_DMAChannel_t spi2_tx_dmas[] = { {&dma1_stream4, 0}, {0} };
const STM32_DMAChannel_t spi2_rx_dmas[] = { {&dma1_stream3, 0}, {0} };

const STM32_GPIO_t* spi3_nss_gpios[] = { &pa4, &pa15, nullptr };
const STM32_GPIO_t* spi3_sck_gpios[] = { &pb3, &pc10, nullptr };
const STM32_GPIO_t* spi3_miso_gpios[] = { &pb4, &pc11, nullptr };
const STM32_GPIO_t* spi3_mosi_gpios[] = { &pb5, &pc12, nullptr };
const STM32_DMAChannel_t spi3_tx_dmas[] = { {&dma1_stream5, 0}, {&dma1_stream7, 0}, {0} };
const STM32_DMAChannel_t spi3_rx_dmas[] = { {&dma1_stream0, 0}, {&dma1_stream2, 0}, {0} };

STM32_SPI_t spi1(SPI1, spi1_sck_gpios, spi1_miso_gpios, spi1_mosi_gpios, GPIO_AF5_SPI1, spi1_tx_dmas, spi1_rx_dmas);
STM32_SPI_t spi2(SPI2, spi2_sck_gpios, spi2_miso_gpios, spi2_mosi_gpios, GPIO_AF5_SPI2, spi2_tx_dmas, spi2_rx_dmas);
STM32_SPI_t spi3(SPI3, spi3_sck_gpios, spi3_miso_gpios, spi3_mosi_gpios, GPIO_AF6_SPI3, spi3_tx_dmas, spi3_rx_dmas);
