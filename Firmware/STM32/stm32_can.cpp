
#include "stm32_can.hpp"
#include "stm32_system.h"

bool STM32_CAN_t::setup(STM32_GPIO_t* rx_gpio, STM32_GPIO_t* tx_gpio) {
    if (hcan.Instance == CAN1)
        __HAL_RCC_CAN1_CLK_ENABLE();
    else if (hcan.Instance == CAN2)
        __HAL_RCC_CAN2_CLK_ENABLE();
    else
        return false;

    hcan.Init.Prescaler = 8;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = ENABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
        return false;

    if (rx_gpio)
        if (!rx_gpio->setup_alternate_function(rx_gpios, gpio_af, GPIO_t::NO_PULL, GPIO_t::VERY_FAST))
            return false;
    if (tx_gpio)
        if (!tx_gpio->setup_alternate_function(tx_gpios, gpio_af, GPIO_t::NO_PULL, GPIO_t::VERY_FAST))
            return false;

    return true;
}

bool STM32_CAN_t::enable_interrupts(uint8_t priority) {
    if (hcan.Instance == CAN1) {
        enable_interrupt(CAN1_TX_IRQn, priority);
        enable_interrupt(CAN1_RX0_IRQn, priority);
        enable_interrupt(CAN1_RX1_IRQn, priority);
        enable_interrupt(CAN1_SCE_IRQn, priority);
    } else if (hcan.Instance == CAN2) {
        enable_interrupt(CAN2_TX_IRQn, priority);
        enable_interrupt(CAN2_RX0_IRQn, priority);
        enable_interrupt(CAN2_RX1_IRQn, priority);
        enable_interrupt(CAN2_SCE_IRQn, priority);
    } else {
        return false;
    }

    return true;
}

extern "C" {
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void CAN1_RX1_IRQHandler(void);
void CAN1_SCE_IRQHandler(void);
void CAN2_TX_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);
void CAN2_RX1_IRQHandler(void);
void CAN2_SCE_IRQHandler(void);
}

/** @brief Entrypoint for the CAN1 TX interrupts. */
void CAN1_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can1.hcan);
}

/** @brief Entrypoint for the CAN1 RX0 interrupts. */
void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can1.hcan);
}

/** @brief Entrypoint for the CAN1 RX1 interrupt. */
void CAN1_RX1_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can1.hcan);
}

/** @brief Entrypoint for the CAN1 SCE interrupt. */
void CAN1_SCE_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can1.hcan);
}

/** @brief Entrypoint for the CAN2 TX interrupts. */
void CAN2_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can2.hcan);
}

/** @brief Entrypoint for the CAN2 RX0 interrupts. */
void CAN2_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can2.hcan);
}

/** @brief Entrypoint for the CAN2 RX1 interrupt. */
void CAN2_RX1_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can2.hcan);
}

/** @brief Entrypoint for the CAN2 SCE interrupt. */
void CAN2_SCE_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can2.hcan);
}


/* Peripheral definitions ----------------------------------------------------*/

const STM32_GPIO_t* can1_rx_gpios[] = { &pa11, &pb8, &pd0, &pi9, nullptr };
const STM32_GPIO_t* can1_tx_gpios[] = { &pa12, &pb9, &pd1, &ph13, nullptr };

const STM32_GPIO_t* can2_rx_gpios[] = { &pb5, &pb12, nullptr };
const STM32_GPIO_t* can2_tx_gpios[] = { &pb6, &pb13, nullptr };

STM32_CAN_t can1(CAN1, can1_rx_gpios, can1_tx_gpios, GPIO_AF9_CAN1);
STM32_CAN_t can2(CAN2, can2_rx_gpios, can2_tx_gpios, GPIO_AF9_CAN2);
