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
ODriveCAN::ODriveCAN(CAN_HandleTypeDef *handle, ODriveCAN::Config_t &config)
    : handle_{handle},
      config_{config} {
}

void ODriveCAN::can_server_thread() {
    static uint32_t counter = 0;
    for (;;) {
        CAN_message_t txmsg;
        txmsg.id = 0x100;
        txmsg.len = 4;
        txmsg.isExt = false;

        txmsg.buf[0] = counter >> 24;
        txmsg.buf[1] = counter >> 16;
        txmsg.buf[2] = counter >> 8;
        txmsg.buf[3] = counter;

        counter++;
        write(txmsg);
        osDelay(10);
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
            handle_->Init.Prescaler = 16;  // 21 TQ's
            handle_->Init.TimeSeg1 = CAN_BS1_16TQ;
            handle_->Init.TimeSeg2 = CAN_BS2_4TQ;
            config_.baud = baudRate;
            break;

        case CAN_BAUD_250K:
            handle_->Init.Prescaler = 8;  // 21 TQ's
            handle_->Init.TimeSeg1 = CAN_BS1_16TQ;
            handle_->Init.TimeSeg2 = CAN_BS2_4TQ;
            config_.baud = baudRate;
            break;

        case CAN_BAUD_500K:
            handle_->Init.Prescaler = 4;  // 21 TQ's
            handle_->Init.TimeSeg1 = CAN_BS1_16TQ;
            handle_->Init.TimeSeg2 = CAN_BS2_4TQ;
            config_.baud = baudRate;
            break;

        case CAN_BAUD_1000K:
            handle_->Init.Prescaler = 2;  // 21 TQ's
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
    if(HAL_CAN_GetTxMailboxesFreeLevel(handle_) > 0)
        HAL_CAN_AddTxMessage(handle_, &header, txmsg.buf, &retTxMailbox);

    return retTxMailbox;
}