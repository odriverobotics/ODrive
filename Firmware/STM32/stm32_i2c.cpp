
#include "stm32_i2c.hpp"
#include "stm32_system.h"

bool STM32_I2C_t::setup_as_slave(uint32_t freq, uint8_t addr, STM32_GPIO_t* scl_gpio, STM32_GPIO_t* sda_gpio, STM32_DMAStream_t* tx_dma, STM32_DMAStream_t* rx_dma) {
    if (hi2c.Instance == I2C1)
        __HAL_RCC_I2C1_CLK_ENABLE();
    else if (hi2c.Instance == I2C2)
        __HAL_RCC_I2C2_CLK_ENABLE();
    else if (hi2c.Instance == I2C3)
        __HAL_RCC_I2C3_CLK_ENABLE();
    else
        return false;

    tx_dma_ = tx_dma;
    rx_dma_ = rx_dma;

    if (tx_dma) {
        if (!tx_dma->init(tx_dmas, DMA_t::MEMORY, DMA_t::PERIPHERAL, DMA_t::ALIGN_8_BIT, DMA_t::LINEAR, DMA_t::LOW)) {
            return false;
        }
        tx_dma->link(hi2c, &I2C_HandleTypeDef::hdmatx);
    }

    if (rx_dma) {
        if (!rx_dma->init(rx_dmas, DMA_t::PERIPHERAL, DMA_t::MEMORY, DMA_t::ALIGN_8_BIT, DMA_t::CIRCULAR, DMA_t::LOW)) {
            return false;
        }
        rx_dma->link(hi2c, &I2C_HandleTypeDef::hdmarx);
    }

    if (scl_gpio)
        if (!scl_gpio->setup_alternate_function(scl_gpios, gpio_af, GPIO_t::PULL_UP, GPIO_t::VERY_FAST, true))
            return false;
    if (sda_gpio)
        if (!sda_gpio->setup_alternate_function(sda_gpios, gpio_af, GPIO_t::PULL_UP, GPIO_t::VERY_FAST, true))
            return false;

    hi2c.Init.ClockSpeed = freq;
    hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c.Init.OwnAddress1 = addr << 1;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.OwnAddress2 = 0;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c) != HAL_OK)
        return false;

    return true;
}

bool STM32_I2C_t::enable_interrupts(uint8_t priority) {
    if (hi2c.Instance == I2C1) {
        enable_interrupt(I2C1_EV_IRQn, priority);
        enable_interrupt(I2C1_ER_IRQn, priority);
    } else if (hi2c.Instance == I2C2) {
        enable_interrupt(I2C2_EV_IRQn, priority);
        enable_interrupt(I2C2_ER_IRQn, priority);
    } else if (hi2c.Instance == I2C3) {
        enable_interrupt(I2C3_EV_IRQn, priority);
        enable_interrupt(I2C3_ER_IRQn, priority);
    } else {
        return false;
    }

    return (!tx_dma_ || tx_dma_->enable_interrupts(priority))
        && (!rx_dma_ || rx_dma_->enable_interrupts(priority));
}

/** @brief This function handles I2C1 event interrupt. */
void I2C1_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&i2c1.hi2c);
}

/** @brief This function handles I2C1 error interrupt. */
void I2C1_ER_IRQHandler(void) {
    HAL_I2C_ER_IRQHandler(&i2c1.hi2c);
}

/** @brief This function handles I2C1 event interrupt. */
void I2C2_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&i2c2.hi2c);
}

/** @brief This function handles I2C1 error interrupt. */
void I2C2_ER_IRQHandler(void) {
    HAL_I2C_ER_IRQHandler(&i2c2.hi2c);
}

/** @brief This function handles I2C1 event interrupt. */
void I2C3_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&i2c3.hi2c);
}

/** @brief This function handles I2C1 error interrupt. */
void I2C3_ER_IRQHandler(void) {
    HAL_I2C_ER_IRQHandler(&i2c3.hi2c);
}


const STM32_GPIO_t* i2c1_scl_gpios[] = { &pb6, &pb8, nullptr };
const STM32_GPIO_t* i2c1_sda_gpios[] = { &pb7, &pb9, nullptr };
const STM32_GPIO_t* i2c1_smba_gpios[] = { &pb5, nullptr };
const STM32_DMAChannel_t i2c1_tx_dmas[] = { {&dma1_stream6, 1}, {&dma1_stream7, 1}, {0} };
const STM32_DMAChannel_t i2c1_rx_dmas[] = { {&dma1_stream0, 1}, {&dma1_stream5, 1}, {0} };

const STM32_GPIO_t* i2c2_scl_gpios[] = { &pb10, &pf1, &ph4, nullptr };
const STM32_GPIO_t* i2c2_sda_gpios[] = { &pb11, &pf0, &ph5, nullptr };
const STM32_GPIO_t* i2c2_smba_gpios[] = { &pb12, &pf2, &ph6, nullptr };
const STM32_DMAChannel_t i2c2_tx_dmas[] = { {&dma1_stream7, 7}, {0} };
const STM32_DMAChannel_t i2c2_rx_dmas[] = { {&dma1_stream2, 7}, {&dma1_stream3, 7}, {0} };

const STM32_GPIO_t* i2c3_scl_gpios[] = { &pa8, &ph7, nullptr };
const STM32_GPIO_t* i2c3_sda_gpios[] = { &pc9, &ph8, nullptr };
const STM32_GPIO_t* i2c3_smba_gpios[] = { &pa9, &ph9, nullptr };
const STM32_DMAChannel_t i2c3_tx_dmas[] = { {&dma1_stream2, 3}, {0} };
const STM32_DMAChannel_t i2c3_rx_dmas[] = { {&dma1_stream4, 3}, {0} };

STM32_I2C_t i2c1(I2C1, i2c1_scl_gpios, i2c1_sda_gpios, GPIO_AF4_I2C1, i2c1_tx_dmas, i2c1_rx_dmas);
STM32_I2C_t i2c2(I2C2, i2c2_scl_gpios, i2c2_sda_gpios, GPIO_AF4_I2C2, i2c2_tx_dmas, i2c2_rx_dmas);
STM32_I2C_t i2c3(I2C3, i2c3_scl_gpios, i2c3_sda_gpios, GPIO_AF4_I2C3, i2c3_tx_dmas, i2c3_rx_dmas);
