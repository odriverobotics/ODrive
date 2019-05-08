
#include "stm32_can.hpp"
#include "stm32_system.h"

bool STM32_CAN_t::init(STM32_GPIO_t* rx_gpio, STM32_GPIO_t* tx_gpio) {
    if (hcan.Instance == CAN1)
        __HAL_RCC_CAN1_CLK_ENABLE();
    else if (hcan.Instance == CAN2)
        __HAL_RCC_CAN2_CLK_ENABLE();
    else
        return false;

    if (rx_gpio)
        if (!rx_gpio->setup_alternate_function(rx_gpios, gpio_af, GPIO_t::NO_PULL, GPIO_t::VERY_FAST))
            return false;
    if (tx_gpio)
        if (!tx_gpio->setup_alternate_function(tx_gpios, gpio_af, GPIO_t::NO_PULL, GPIO_t::VERY_FAST))
            return false;

    return true;
}

bool STM32_CAN_t::config(CAN_baudrate_t baudrate) {
    // Set one of only a few common baud rates.  CAN doesn't do arbitrary baud rates well due to the time-quanta issue.
    // 21 TQ allows for easy sampling at exactly 80% (recommended by Vector Informatik GmbH for high reliability systems)
    // Conveniently, the CAN peripheral's 42MHz clock lets us easily create 21TQs for all common baud rates
    // TODO: derive prescaler from CAN frequency at runtime
    switch (baudrate) {
        case CAN_BAUD_125K: hcan.Init.Prescaler = 16; break; // 21 TQ's
        case CAN_BAUD_250K: hcan.Init.Prescaler = 8; break; // 21 TQ's
        case CAN_BAUD_500K: hcan.Init.Prescaler = 4; break; // 21 TQ's
        case CAN_BAUD_1000K: hcan.Init.Prescaler = 2; break; // 21 TQ's
        default: return false;
    }

    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE; // recover automatically from bus off error condition
    hcan.Init.AutoWakeUp = ENABLE;
    hcan.Init.AutoRetransmission = ENABLE; // retransmit message until succeeded, according to CAN standard
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    return HAL_CAN_Init(&hcan) == HAL_OK;
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

bool STM32_CAN_t::start() {
    return HAL_CAN_Start(&hcan) == HAL_OK
        && HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK
        && HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) == HAL_OK;
}

bool STM32_CAN_t::subscribe(uint32_t id, uint32_t mask, uint32_t fifo, void(*callback)(void* ctx, CAN_message_t& msg), void* ctx) {
    if (filter_banks_in_use_ >= N_RX_FILTER_BANKS) {
        return false;
    }

    if (fifo == 0) {
        if (n_fifo0_subscribers >= sizeof(fifo0_subscribers) / sizeof(fifo0_subscribers[0]))
            return false;
        fifo0_subscribers[n_fifo0_subscribers++].set<void>(callback, ctx);
    } else if (fifo == 1) {
        if (n_fifo1_subscribers >= sizeof(fifo1_subscribers) / sizeof(fifo1_subscribers[0]))
            return false;
        fifo1_subscribers[n_fifo1_subscribers++].set<void>(callback, ctx);
    }

    CAN_FilterTypeDef filter;
    filter.FilterActivation = ENABLE;
    filter.FilterBank = filter_banks_in_use_++;
    filter.FilterFIFOAssignment = fifo;
    filter.FilterIdHigh = (id >> 16) & 0xffff;
    filter.FilterIdLow = id & 0xffff;
    filter.FilterMaskIdHigh = (mask >> 16) & 0xffff;
    filter.FilterMaskIdLow = mask & 0xffff;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    return HAL_CAN_ConfigFilter(&hcan, &filter) == HAL_OK;
}

bool STM32_CAN_t::send(CAN_message_t& msg, uint32_t* mailbox) {
    uint32_t dummy = 0;

    if (HAL_CAN_GetError(&hcan) == HAL_CAN_ERROR_NONE) {
        CAN_TxHeaderTypeDef header;
        header.StdId = msg.id;
        header.ExtId = msg.id;
        header.IDE = msg.isExt ? CAN_ID_EXT : CAN_ID_STD;
        header.RTR = CAN_RTR_DATA;
        header.DLC = msg.len;
        header.TransmitGlobalTime = FunctionalState::DISABLE;

        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
            return false;

        return HAL_CAN_AddTxMessage(&hcan, &header, msg.buf, mailbox ? mailbox : &dummy) == HAL_OK;
    } else {
        return false;
    }
}

bool STM32_CAN_t::handle_rx_message(uint32_t fifo) {
    CAN_message_t msg;
    CAN_RxHeaderTypeDef header;
    if (fifo >= N_RX_FIFOS)
        return false;
    if (HAL_CAN_GetRxMessage(&hcan, fifo, &header, msg.buf) != HAL_OK)
        return false;

    msg.isExt = header.IDE;
    msg.id = msg.isExt ? header.ExtId : header.StdId;  // If it's an extended message, pass the extended ID
    msg.len = header.DLC;
    msg.rtr = header.RTR;
    
    if (header.FilterMatchIndex >= (fifo == 0 ? n_fifo0_subscribers : n_fifo1_subscribers))
        return false;

    Subscriber<CAN_message_t&>& subscriber = (fifo == 0 ? fifo0_subscribers[header.FilterMatchIndex] : fifo1_subscribers[header.FilterMatchIndex]);
    subscriber.invoke(msg);

    return true;
}


/* Interrupt entrypoints -----------------------------------------------------*/

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


/* HAL callbacks -------------------------------------------------------------*/

extern "C" {

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (hcan->Instance == CAN1) {
        can1.handle_rx_message(0);
    } else if (hcan->Instance == CAN2) {
        can2.handle_rx_message(0);
    }
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    if (hcan->Instance == CAN1) {
        can1.handle_rx_message(1);
    } else if (hcan->Instance == CAN2) {
        can2.handle_rx_message(1);
    }
}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_ResetError(hcan);
}

} // extern "C"


/* Peripheral definitions ----------------------------------------------------*/

const STM32_GPIO_t* can1_rx_gpios[] = { &pa11, &pb8, &pd0, &pi9, nullptr };
const STM32_GPIO_t* can1_tx_gpios[] = { &pa12, &pb9, &pd1, &ph13, nullptr };

const STM32_GPIO_t* can2_rx_gpios[] = { &pb5, &pb12, nullptr };
const STM32_GPIO_t* can2_tx_gpios[] = { &pb6, &pb13, nullptr };

STM32_CAN_t can1(CAN1, can1_rx_gpios, can1_tx_gpios, GPIO_AF9_CAN1);
STM32_CAN_t can2(CAN2, can2_rx_gpios, can2_tx_gpios, GPIO_AF9_CAN2);
