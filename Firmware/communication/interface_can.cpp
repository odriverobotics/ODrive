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
#include <stm32f4xx_hal.h>
#include <cmsis_os.h>

#define CAN_HEARTBEAT_INTERVAL  1000 // [ms]
#define CAN_HEARTBEAT_MARGIN    10 // maximum time that a heartbeat message can be delayed until we stop sending other messages [ms]

// defined in can.c
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

static CAN_context* ctxs[3] = { nullptr, nullptr, nullptr };

struct CAN_context* get_can_ctx(CAN_HandleTypeDef *hcan) {
#if defined(CAN1)
    if (hcan->Instance == CAN1) return ctxs[0];
#endif
#if defined(CAN2)
    if (hcan->Instance == CAN2) return ctxs[1];
#endif
#if defined(CAN3)
    if (hcan->Instance == CAN3) return ctxs[2];
#endif
    return nullptr;
}


void consider_node_id_in_use(CAN_context* ctx, uint8_t node_id) {
    ctx->node_ids_in_use_0[node_id >> 5] |= (1 << (node_id & 0x1f));
}

bool is_node_id_in_use(CAN_context* ctx, uint32_t node_id) {
    if (node_id == 0) // node ID 0 is reserved (is it though?)
        return true;
    return (ctx->node_ids_in_use_0[node_id >> 5] & (1 << (node_id & 0x1f)))
        || (ctx->node_ids_in_use_1[node_id >> 5] & (1 << (node_id & 0x1f)));
}

bool select_another_node_id(CAN_context* ctx) {
    ctx->node_id_expiry = osKernelSysTick() - 1;

    // Find a new node ID that is not in use
    for (uint8_t i = 0; i < 32; i++) {
        // Each time we select a new node ID, we use the next byte from the serial
        // number to get advance the node ID.
        uint8_t poor_mans_random_byte = ((uint8_t*)ctx->serial_number)[ctx->node_id_rng_state];
        if (++(ctx->node_id_rng_state) >= sizeof(ctx->serial_number))
            ctx->node_id_rng_state = 0;
        ctx->node_id = calc_crc<uint8_t, 1>(ctx->node_id, poor_mans_random_byte);
        if (!is_node_id_in_use(ctx, ctx->node_id))
            return true;
    }
    return false;
}


void server_thread(CAN_context* ctx) {
    uint32_t next_1s_tick = osKernelSysTick() + 1000;
    for (;;) {
        if (deadline_to_timeout(next_1s_tick) == 0)
            
        // wait until either the next heartbeat is due or a hearbeat was requested
        // by releasing the semaphore
        osSemaphoreWait(ctx->sem_send_heartbeat, deadline_to_timeout(next_1s_tick));
        if (!is_in_the_future(next_1s_tick))
            memcpy(ctx->node_ids_in_use_1, ctx->node_ids_in_use_0, sizeof(ctx->node_ids_in_use_1));
        next_1s_tick += 1000;
        if (!is_in_the_future(next_1s_tick))
            next_1s_tick = osKernelSysTick(); // fast-forward if we missed several 1 second ticks

        if (is_node_id_in_use(ctx, ctx->node_id)) {
            if (!select_another_node_id(ctx))
                continue;
            else
                next_1s_tick += ctx->node_id; // shift the 1s tick by a bit
        }

        uint8_t data[8];
        //uint8_t data[] = { ctx->node_id }; // this would be the correct data for CANopen - TODO: make it compatible
        *(uint64_t*)data = ctx->serial_number;

        CAN_TxHeaderTypeDef header = {
            .StdId = 0x700u + ctx->node_id,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = sizeof(data),
            .TransmitGlobalTime = DISABLE
        };
        HAL_CAN_AddTxMessage(ctx->handle, &header, data, &ctx->last_heartbeat_mailbox);
    }
}

bool start_can_server(CAN_context& ctx, CAN_TypeDef *port, uint64_t serial_number) {
    //MX_CAN1_Init(); // TODO: flatten
#if defined(CAN1)
    if (port == CAN1) ctx.handle = &hcan1, ctxs[0] = &ctx; else
#endif
#if defined(CAN2)
    // TODO: move CubeMX stuff into this file so all symbols are defined
    //if (port == CAN2) ctx.handle = &hcan2, ctxs[1] = &ctx; else
#endif
#if defined(CAN3)
    if (port == CAN3) ctx.handle = &hcan3, ctxs[2] = &ctx; else
#endif
    return false; // fail if none of the above checks matched

    HAL_StatusTypeDef status;

    ctx.node_id = calc_crc<uint8_t, 1>(0, (const uint8_t*)UID_BASE, 12);
    ctx.serial_number = serial_number;
    osSemaphoreDef(sem_send_heartbeat);
    ctx.sem_send_heartbeat = osSemaphoreCreate(osSemaphore(sem_send_heartbeat), 1);
    osSemaphoreWait(ctx.sem_send_heartbeat, 0);

    //// Set up heartbeat filter
    CAN_FilterTypeDef sFilterConfig = {
        .FilterIdHigh = ((0x700u + ctx.node_id) << 5) | (0x0 << 2), // own heartbeat (standard ID, no RTR)
        .FilterIdLow = (0x700u << 5) | (0x0 << 2), // any heartbeat (standard ID, no RTR)
        .FilterMaskIdHigh = (0x7ffu << 5) | (0x3 << 2),
        .FilterMaskIdLow = (0x780u << 5) | (0x3 << 2),
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_16BIT, // two 16-bit filters
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 0
    };
    status = HAL_CAN_ConfigFilter(ctx.handle, &sFilterConfig);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_Start(ctx.handle);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_ActivateNotification(ctx.handle,
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
    
    server_thread(&ctx);
    return true;
}

void tx_complete_callback(CAN_HandleTypeDef *hcan, uint8_t mailbox_idx) {
    CAN_context *ctx = get_can_ctx(hcan);
    if (!ctx) return;
    ctx->tx_msg_cnt++;
    if (mailbox_idx == ctx->last_heartbeat_mailbox) {
        // we succeeded in sending a heartbeat
        // now we're allowed to send messages for the next second plus a small margin
        ctx->node_id_expiry = osKernelSysTick() + CAN_HEARTBEAT_INTERVAL + CAN_HEARTBEAT_MARGIN;
    }
}

void tx_aborted_callback(CAN_HandleTypeDef *hcan, uint8_t mailbox_idx) {
    //__asm volatile ("bkpt");
    if (!get_can_ctx(hcan))
        return;
    get_can_ctx(hcan)->TxMailboxAbortCallbackCnt++;
}

void tx_error(CAN_context *ctx, uint8_t mailbox_idx) {
    if (mailbox_idx == ctx->last_heartbeat_mailbox) {
        // Consider the node ID in use
        consider_node_id_in_use(ctx, ctx->node_id);
        // Try to find a new node ID that is not in use and immediately
        // resend heartbeat if we find one
        if (select_another_node_id(ctx))
            osSemaphoreRelease(ctx->sem_send_heartbeat);
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { tx_complete_callback(hcan, 0); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { tx_complete_callback(hcan, 1); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { tx_complete_callback(hcan, 2); }
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) { tx_aborted_callback(hcan, 0); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) { tx_aborted_callback(hcan, 1); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) { tx_aborted_callback(hcan, 2); }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_context *ctx = get_can_ctx(hcan);
    if (!ctx) return;
    ctx->received_msg_cnt++;

    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);
    if (status != HAL_OK) {
        ctx->unexpected_errors++;
        return;
    }

    uint8_t node_id = header.StdId & 0x07fu;
    if ((header.StdId & 0x780u) == 0x700u) {
        ctx->received_ack++;
        consider_node_id_in_use(ctx, node_id);
    } else {
        ctx->unhandled_messages++;
    }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) { if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo0FullCallbackCnt++; }

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) { if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo1MsgPendingCallbackCnt++; }
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) { if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo1FullCallbackCnt++; }
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) { if (get_can_ctx(hcan)) get_can_ctx(hcan)->SleepCallbackCnt++; }
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) { if (get_can_ctx(hcan)) get_can_ctx(hcan)->WakeUpFromRxMsgCallbackCnt++; }

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    //__asm volatile ("bkpt");
    CAN_context *ctx = get_can_ctx(hcan);
    if (!ctx) return;
    volatile uint32_t original_error = hcan->ErrorCode;
    (void) original_error;

    // handle transmit errors in all three mailboxes
    if (hcan->ErrorCode & HAL_CAN_ERROR_TX_ALST0) {
        SET_BIT(hcan->Instance->sTxMailBox[0].TIR, CAN_TI0R_TXRQ);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_ALST0;
    } else if (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0) {
        tx_error(ctx, 0);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_EWG;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_ACK;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_TERR0;
    }

    if (hcan->ErrorCode & HAL_CAN_ERROR_TX_ALST1) {
        SET_BIT(hcan->Instance->sTxMailBox[1].TIR, CAN_TI1R_TXRQ);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_ALST1;
    } else if (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR1) {
        tx_error(ctx, 1);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_EWG;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_ACK;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_TERR1;
    }

    if (hcan->ErrorCode & HAL_CAN_ERROR_TX_ALST2) {
        SET_BIT(hcan->Instance->sTxMailBox[2].TIR, CAN_TI2R_TXRQ);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_ALST2;
    } else if (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR2) {
        tx_error(ctx, 2);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_EWG;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_ACK;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_TERR2;
    }

    if (hcan->ErrorCode)
        ctx->unexpected_errors++;
}

