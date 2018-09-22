/*
*
* Zero-config node ID negotiation
* -------------------------------
*
* A heartbeat message is a message with a 8 byte unique serial number as payload.
* A regular message is any message that is not a heartbeat message.
*
* All nodes MUST obey these four rules:
*
* a) At a given point in time, a node MUST consider a node ID taken (by others)
*   if any of the following is true:
*     - the node received a (not self-emitted) heartbeat message with that node ID
*       within the last second
*     - the node attempted and failed at sending a heartbeat message with that
*       node ID within the last second (failed in the sense of not ACK'd)
*
* b) At a given point in time, a node MUST NOT consider a node ID self-assigned
*   if, within the last second, it did not succeed in sending a heartbeat
*   message with that node ID.
*
* c) At a given point in time, a node MUST NOT send any heartbeat message with
*   a node ID that is taken.
*
* d) At a given point in time, a node MUST NOT send any regular message with
*   a node ID that is not self-assigned.
*
* Hardware allocation
* -------------------
*   RX FIFO0:
*       - filter bank 0: heartbeat messages
*/

#include "interface_can.hpp"
#include "fibre/crc.hpp"
#include "utils.h"

#include <can.h>
#include <cmsis_os.h>
#include <stm32f4xx_hal.h>

// Constructor is called by communication.cpp and the handle is assigned appropriately
ODriveCAN::ODriveCAN(CAN_HandleTypeDef *handle, CANConfig_t &config)
    : handle_{handle},
      config_{config} {
}

static void can_server_thread_wrapper(void *ctx) {
    reinterpret_cast<ODriveCAN *>(ctx)->can_server_thread();
    reinterpret_cast<ODriveCAN *>(ctx)->thread_id_valid_ = false;
}

void ODriveCAN::can_server_thread() {
    for (;;) {
        osDelay(10);
    }
}

bool ODriveCAN::start_can_server() {
    HAL_StatusTypeDef status;

    set_baud_rate(config_.baud);
    status = HAL_CAN_Init(handle_);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_Start(handle_);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_ActivateNotification(handle_,
                                          CAN_IT_TX_MAILBOX_EMPTY |
                                              CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | /* we probably only want this */
                                              CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO1_FULL |
                                              CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN |
                                              CAN_IT_WAKEUP | CAN_IT_SLEEP_ACK |
                                              CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                                              CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
                                              CAN_IT_ERROR);
    if (status != HAL_OK)
        return false;

    osThreadDef(can_server_thread_def, can_server_thread_wrapper, osPriorityNormal, 0, 512);
    thread_id_ = osThreadCreate(osThread(can_server_thread_def), this);
    thread_id_valid_ = true;

    return true;
}

void ODriveCAN::set_baud_rate(uint32_t baudRate) {
    switch (baudRate) {
        case CAN_BAUD_125K:
            handle_->Init.Prescaler = 21;   // 16 TQ's
            handle_->Init.TimeSeg1 = CAN_BS1_12TQ;
            handle_->Init.TimeSeg2 = CAN_BS2_3TQ;
            config_.baud = baudRate;
            break;
        case CAN_BAUD_250K:
            handle_->Init.Prescaler = 8;    // 21 TQ's
            handle_->Init.TimeSeg1 = CAN_BS1_16TQ;
            handle_->Init.TimeSeg2 = CAN_BS2_4TQ;
            config_.baud = baudRate;
            break;
        case CAN_BAUD_500K:
            handle_->Init.Prescaler = 4;    // 21 TQ's
            handle_->Init.TimeSeg1 = CAN_BS1_16TQ;
            handle_->Init.TimeSeg2 = CAN_BS2_4TQ;
            config_.baud = baudRate;
        case CAN_BAUD_1000K:
            handle_->Init.Prescaler = 2;    // 21 TQ's
            handle_->Init.TimeSeg1 = CAN_BS1_16TQ;
            handle_->Init.TimeSeg2 = CAN_BS2_4TQ;
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