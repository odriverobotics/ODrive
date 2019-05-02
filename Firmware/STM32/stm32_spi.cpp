
#include "stm32_spi.hpp"
#include "stm32_system.h"

bool STM32_SPI_t::init(STM32_GPIO_t* sck_gpio, STM32_GPIO_t* miso_gpio, STM32_GPIO_t* mosi_gpio, STM32_DMAStream_t* tx_dma, STM32_DMAStream_t* rx_dma, bool clock_polarity) {
    if (hspi.Instance == SPI1)
        __HAL_RCC_SPI1_CLK_ENABLE();
    else if (hspi.Instance == SPI2)
        __HAL_RCC_SPI2_CLK_ENABLE();
    else if (hspi.Instance == SPI3)
        __HAL_RCC_SPI3_CLK_ENABLE();
    else
        return false;
    
    tx_dma_ = tx_dma;
    rx_dma_ = rx_dma;

    if (tx_dma) {
        if (!tx_dma->init(tx_dmas, DMA_t::MEMORY, DMA_t::PERIPHERAL, DMA_t::ALIGN_8_BIT, DMA_t::LINEAR, DMA_t::LOW)) {
            return false;
        }
        tx_dma->link(hspi, &SPI_HandleTypeDef::hdmatx);
    }

    if (rx_dma) {
        if (!rx_dma->init(rx_dmas, DMA_t::PERIPHERAL, DMA_t::MEMORY, DMA_t::ALIGN_8_BIT, DMA_t::LINEAR, DMA_t::LOW)) {
            return false;
        }
        rx_dma->link(hspi, &SPI_HandleTypeDef::hdmarx);
    }

    hspi.Init.Mode = SPI_MODE_MASTER;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_8BIT; // TODO: revert
    hspi.Init.CLKPolarity = clock_polarity ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.NSS = SPI_NSS_SOFT;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // TODO: make configurable
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
        if (!miso_gpio->setup_alternate_function(miso_gpios, gpio_af, GPIO_t::PULL_UP, GPIO_t::VERY_FAST))
            return false;
    if (mosi_gpio)
        if (!mosi_gpio->setup_alternate_function(mosi_gpios, gpio_af, GPIO_t::NO_PULL, GPIO_t::VERY_FAST))
            return false;

    return true;
}

bool STM32_SPI_t::enable_interrupts(uint8_t priority) {
    enable_interrupt(get_irq_number(), priority);

    return (!rx_dma_ || rx_dma_->enable_interrupts(priority));
}

bool STM32_SPI_t::enqueue(SPI_task_t* task) {
    *task_queue_tail_ = task;
    task_queue_tail_ = &task->next;
    task->next = nullptr;
    return true;
}

bool STM32_SPI_t::start() {
    if (!tx_dma_ || !rx_dma_) {
        return false; // technically the SPI would work in TX-only or RX-only mode but this is not implemented
    }

    if (next_task_) {
        if (next_task_->n_cs_gpio)
            next_task_->n_cs_gpio->write(false);
        rx_dma_->hdma.Parent = this;
        rx_dma_->hdma.XferCpltCallback = [](DMA_HandleTypeDef *hdma){
            ((STM32_SPI_t*)hdma->Parent)->handle_txrx_complete();
        };
        // NOTE: need modded HAL_SPI_TransmitReceive_DMA to prevent overriding XferCpltCallback
        if (HAL_SPI_TransmitReceive_DMA(&hspi, next_task_->tx_buf, next_task_->rx_buf, next_task_->length) != HAL_OK)
            return false;
    }

    return true;
}

bool STM32_SPI_t::handle_txrx_complete() {
    if (next_task_->n_cs_gpio)
        next_task_->n_cs_gpio->write(true);
    next_task_ = next_task_->next;
    if (next_task_) {
        if (next_task_->n_cs_gpio)
            next_task_->n_cs_gpio->write(false);
        
        // "Recharge" receiving DMA
        rx_dma_->hdma.Instance->M0AR = reinterpret_cast<uint32_t>(next_task_->rx_buf);
        rx_dma_->hdma.Instance->NDTR = next_task_->length;
        rx_dma_->hdma.Instance->CR |= DMA_SxCR_TCIE | DMA_SxCR_EN;

        // "Recharge" transmitting DMA
        tx_dma_->hdma.Instance->M0AR = reinterpret_cast<uint32_t>(next_task_->tx_buf);
        tx_dma_->hdma.Instance->NDTR = next_task_->length;

        // clear TX transfer complete interrupt flag
        ((DMA_Base_Registers *)tx_dma_->hdma.StreamBaseAddress)->IFCR = DMA_FLAG_TCIF0_4 << tx_dma_->hdma.StreamIndex;

        // kick off next transmission by re-enabling TX DMA
        tx_dma_->hdma.Instance->CR |= DMA_SxCR_EN;
        return true;
    } else {
        task_queue_tail_ = &next_task_;
        return HAL_SPI_DMAStop(&hspi) == HAL_OK;
    }
}

bool STM32_SPI_t::handle_error() {
    errors_++;
    return true; // TODO: error handling
}

/* Interrupt entrypoints -----------------------------------------------------*/

extern "C" {

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

} // extern "C"


/* HAL callbacks -------------------------------------------------------------*/

static STM32_SPI_t* all_spis[] = { &spi1, &spi2, &spi3 }; // TODO: this prevents compile time garbage collection

//extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
//    for (size_t i = 0; i < sizeof(all_spis) / sizeof(all_spis[0]); ++i) {
//        if (&all_spis[i]->hspi == hspi) {
//            all_spis[i]->handle_txrx_complete();
//        }
//    }
//}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    for (size_t i = 0; i < sizeof(all_spis) / sizeof(all_spis[0]); ++i) {
        if (&all_spis[i]->hspi == hspi) {
            all_spis[i]->handle_error();
        }
    }
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
