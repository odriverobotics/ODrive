#include "interface_can.hpp"

#include "fibre/crc.hpp"
#include "freertos_vars.h"
#include "utils.hpp"

#include <can.h>
#include <cmsis_os.h>

// Specific CAN Protocols
#include "can_simple.hpp"

// Safer context handling via maps instead of arrays
// #include <unordered_map>
// std::unordered_map<CAN_HandleTypeDef *, ODriveCAN *> ctxMap;

// Constructor is called by communication.cpp and the handle is assigned appropriately
ODriveCAN::ODriveCAN(ODriveCAN::Config_t &config, CAN_HandleTypeDef *handle)
    : config_{config},
      handle_{handle} {
    // ctxMap[handle_] = this;
}

void ODriveCAN::can_server_thread() {
    for (;;) {
        uint32_t status = HAL_CAN_GetError(handle_);
        if (status == HAL_CAN_ERROR_NONE) {
            can_Message_t rxmsg;

            osSemaphoreWait(sem_can, 10);  // Poll every 10ms regardless of sempahore status
            while (available()) {
                read(rxmsg);
                switch (config_.protocol) {
                    case PROTOCOL_SIMPLE:
                        CANSimple::handle_can_message(rxmsg);
                        break;
                }
            }
            HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
        } else {
            if (status == HAL_CAN_ERROR_TIMEOUT) {
                HAL_CAN_ResetError(handle_);
                status = HAL_CAN_Start(handle_);
                if (status == HAL_OK)
                    status = HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
            }
        }
    }
}

static void can_server_thread_wrapper(void *ctx) {
    reinterpret_cast<ODriveCAN *>(ctx)->can_server_thread();
    reinterpret_cast<ODriveCAN *>(ctx)->thread_id_valid_ = false;
}

bool ODriveCAN::start_can_server() {
    HAL_StatusTypeDef status;

    set_baud_rate(config_.baud_rate);

    status = HAL_CAN_Init(handle_);

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

    status = HAL_CAN_Start(handle_);
    if (status == HAL_OK)
        status = HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);

    osThreadDef(can_server_thread_def, can_server_thread_wrapper, osPriorityNormal, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(can_server_thread_def), this);
    thread_id_valid_ = true;

    return status;
}

// Send a CAN message on the bus
uint32_t ODriveCAN::write(can_Message_t &txmsg) {
    if (HAL_CAN_GetError(handle_) == HAL_CAN_ERROR_NONE) {
        CAN_TxHeaderTypeDef header;
        header.StdId = txmsg.id;
        header.ExtId = txmsg.id;
        header.IDE = txmsg.isExt ? CAN_ID_EXT : CAN_ID_STD;
        header.RTR = CAN_RTR_DATA;
        header.DLC = txmsg.len;
        header.TransmitGlobalTime = FunctionalState::DISABLE;

        uint32_t retTxMailbox = 0;
        if (HAL_CAN_GetTxMailboxesFreeLevel(handle_) > 0)
            HAL_CAN_AddTxMessage(handle_, &header, txmsg.buf, &retTxMailbox);

        return retTxMailbox;
    } else {
        return -1;
    }
}

uint32_t ODriveCAN::available() {
    return (HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO0) + HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO1));
}

bool ODriveCAN::read(can_Message_t &rxmsg) {
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
    rxmsg.rtr = header.RTR;

    return validRead;
}

// Set one of only a few common baud rates.  CAN doesn't do arbitrary baud rates well due to the time-quanta issue.
// 21 TQ allows for easy sampling at exactly 80% (recommended by Vector Informatik GmbH for high reliability systems)
// Conveniently, the CAN peripheral's 42MHz clock lets us easily create 21TQs for all common baud rates
void ODriveCAN::set_baud_rate(uint32_t baudRate) {
    switch (baudRate) {
        case CAN_BAUD_125K:
            handle_->Init.Prescaler = 16;  // 21 TQ's
            config_.baud_rate = baudRate;
            reinit_can();
            break;

        case CAN_BAUD_250K:
            handle_->Init.Prescaler = 8;  // 21 TQ's
            config_.baud_rate = baudRate;
            reinit_can();
            break;

        case CAN_BAUD_500K:
            handle_->Init.Prescaler = 4;  // 21 TQ's
            config_.baud_rate = baudRate;
            reinit_can();
            break;

        case CAN_BAUD_1000K:
            handle_->Init.Prescaler = 2;  // 21 TQ's
            config_.baud_rate = baudRate;
            reinit_can();
            break;

        default:
            // baudRate is invalid, so don't accept it.
            break;
    }
}

void ODriveCAN::reinit_can() {
    HAL_CAN_Stop(handle_);
    HAL_CAN_Init(handle_);
    auto status = HAL_CAN_Start(handle_);
    if (status == HAL_OK)
        status = HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void ODriveCAN::set_error(Error error) {
    error_ |= error;
}
// This function is called by each axis.
// It provides an abstraction from the specific CAN protocol in use
void ODriveCAN::send_heartbeat(Axis *axis) {
    // Handle heartbeat message
    if (axis->config_.can_heartbeat_rate_ms > 0) {
        uint32_t now = HAL_GetTick();
        if ((now - axis->last_heartbeat_) >= axis->config_.can_heartbeat_rate_ms) {
            switch (config_.protocol) {
                case PROTOCOL_SIMPLE:
                    CANSimple::send_heartbeat(axis);
                    break;
            }
            axis->last_heartbeat_ = now;
        }
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
