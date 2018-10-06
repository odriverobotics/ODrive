#include "interface_can.hpp"

#include "fibre/crc.hpp"
#include "freertos_vars.h"
#include "utils.h"

#include <can.h>
#include <cmsis_os.h>
#include <stm32f4xx_hal.h>

// Specific CAN Protocols
#include "can_simple.hpp"

#include <odrive_main.h>

// Safer context handling via maps instead of arrays
// #include <unordered_map>
// std::unordered_map<CAN_HandleTypeDef *, ODriveCAN *> ctxMap;

// Constructor is called by communication.cpp and the handle is assigned appropriately
ODriveCAN::ODriveCAN(CAN_HandleTypeDef *handle, ODriveCAN::Config_t &config)
    : handle_{handle},
      config_{config} {
    // ctxMap[handle_] = this;
}

void ODriveCAN::can_server_thread() {
    for (;;) {
        CAN_message_t rxmsg;

        osSemaphoreWait(sem_can, 10);  // Poll every 10ms regardless of sempahore status
        while (available()) {
            read(rxmsg);
            switch (config_.protocol) {
                case CAN_PROTOCOL_SIMPLE:
                    CANSimple::handle_can_message(rxmsg);
                    break;
            }
        }
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

static void can_server_thread_wrapper(void *ctx) {
    reinterpret_cast<ODriveCAN *>(ctx)->can_server_thread();
    reinterpret_cast<ODriveCAN *>(ctx)->thread_id_valid_ = false;
}

bool ODriveCAN::start_can_server() {
    HAL_StatusTypeDef status;

    set_baud_rate(config_.baud);
    status = HAL_CAN_Init(handle_);
    if (status != HAL_OK)
        return false;

    CAN_FilterTypeDef filter;
    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    status = HAL_CAN_ConfigFilter(handle_, &filter);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_Start(handle_);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_ActivateNotification(handle_,
                                          //   CAN_IT_TX_MAILBOX_EMPTY |
                                          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING /* we probably only want this */
                                                                                                    //   CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO1_FULL
                                                                                                    //   CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN |
                                                                                                    //   CAN_IT_WAKEUP | CAN_IT_SLEEP_ACK |
                                                                                                    //   CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                                                                                                    //   CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
                                                                                                    //   | CAN_IT_ERROR
    );
    if (status != HAL_OK)
        return false;

    osThreadDef(can_server_thread_def, can_server_thread_wrapper, osPriorityNormal, 0, 512);
    thread_id_ = osThreadCreate(osThread(can_server_thread_def), this);
    thread_id_valid_ = true;

    return true;
}

// Send a CAN message on the bus
uint32_t ODriveCAN::write(CAN_message_t &txmsg) {
    CAN_TxHeaderTypeDef header;
    header.StdId = txmsg.id;
    header.ExtId = txmsg.id;
    header.IDE = txmsg.isExt ? CAN_ID_EXT : CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = txmsg.len;
    header.TransmitGlobalTime = FunctionalState::DISABLE;

    uint32_t retTxMailbox;
    if (HAL_CAN_GetTxMailboxesFreeLevel(handle_) > 0)
        HAL_CAN_AddTxMessage(handle_, &header, txmsg.buf, &retTxMailbox);

    return retTxMailbox;
}

uint32_t ODriveCAN::available() {
    return (HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO0) + HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO1));
}

bool ODriveCAN::read(CAN_message_t &rxmsg) {
    CAN_RxHeaderTypeDef header;
    bool validRead = false;
    if (HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO0) > 0) {
        HAL_CAN_GetRxMessage(handle_, CAN_RX_FIFO0, &header, rxmsg.buf);
        validRead = true;
    } else if (HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO1) > 0) {
        HAL_CAN_GetRxMessage(handle_, CAN_RX_FIFO1, &header, rxmsg.buf);
        validRead = true;
    }

    rxmsg.isExt = header.IDE;
    rxmsg.id = rxmsg.isExt ? header.ExtId : header.StdId;  // If it's an extended message, pass the extended ID
    rxmsg.len = header.DLC;

    return validRead;
}

// Set one of only a few common baud rates.  CAN doesn't do arbitrary baud rates well due to the time-quanta issue.
// 21 TQ allows for easy sampling at exactly 80% (recommended by Vector Informatik GmbH for high reliability systems)
// Conveniently, the CAN peripheral's 42MHz clock lets us easily create 21TQs for all common baud rates
void ODriveCAN::set_baud_rate(uint32_t baudRate) {
    switch (baudRate) {
        case CAN_BAUD_125K:
            handle_->Init.Prescaler = 16;  // 21 TQ's
            config_.baud = baudRate;
            break;

        case CAN_BAUD_250K:
            handle_->Init.Prescaler = 8;  // 21 TQ's
            config_.baud = baudRate;
            break;

        case CAN_BAUD_500K:
            handle_->Init.Prescaler = 4;  // 21 TQ's
            config_.baud = baudRate;
            break;

        case CAN_BAUD_1000K:
            handle_->Init.Prescaler = 2;  // 21 TQ's
            config_.baud = baudRate;
            break;

        default:
            break;  // baudRate is invalid, so do nothing
    }
}

void ODriveCAN::set_node_id(uint8_t nodeID) {
    // Allow for future nodeID validation by making this a set function
    config_.node_id = nodeID;
}

// This function is called by each axis.  
// It provides an abstraction from the specific CAN protocol in use
void ODriveCAN::send_heartbeat(Axis *axis) {
    // Handle heartbeat message
    uint32_t now = osKernelSysTick();
    if (now - axis->last_heartbeat_ >= 100) {
        switch (config_.protocol) {
            case CAN_PROTOCOL_SIMPLE:
                CANSimple::send_heartbeat(axis);
                break;
        }
        axis->last_heartbeat_ = now;
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
    // osSemaphoreRelease(sem_can);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_ResetError(hcan);
}
