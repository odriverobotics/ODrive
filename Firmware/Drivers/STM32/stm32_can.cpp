
#include "stm32_can.hpp"

bool Stm32Can::init(CAN_InitTypeDef config, uint32_t can_freq) {
    handle_.Init = config;
    can_freq_ = can_freq;

    if (handle_.Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
#ifdef CAN2
    } else if (handle_.Instance == CAN2) {
        __HAL_RCC_CAN2_CLK_ENABLE();
#endif
    } else {
        return false;
    }

    handle_.Init.Prescaler = 8; // this is overridden during start()
    return HAL_CAN_Init(&handle_) == HAL_OK;
}

bool Stm32Can::is_valid_baud_rate(uint32_t nominal_baud_rate, uint32_t data_baud_rate) {
    uint32_t prescaler = can_freq_ / nominal_baud_rate;
    return (nominal_baud_rate == data_baud_rate)
        && (prescaler * nominal_baud_rate == can_freq_);
}

bool Stm32Can::start(uint32_t nominal_baud_rate, uint32_t data_baud_rate, on_event_cb_t on_event, on_error_cb_t on_error) {
    if (nominal_baud_rate != data_baud_rate) {
        return false;
    }

    uint32_t prescaler = can_freq_ / nominal_baud_rate;
    if (prescaler * nominal_baud_rate != can_freq_) {
        return false;
    }

    handle_.Init.Prescaler = prescaler;
    on_event_ = on_event;
    on_error_ = on_error;

    HAL_CAN_ResetError(&handle_);

    bool started = (HAL_CAN_Init(&handle_) == HAL_OK)
                && (HAL_CAN_Start(&handle_) == HAL_OK)
                && (HAL_CAN_ActivateNotification(&handle_, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) == HAL_OK);

    if (!started) {
        return false;
    }

    return false;
}

bool Stm32Can::stop() {
    return HAL_CAN_Stop(&handle_) == HAL_OK;
}

// Send a CAN message on the bus
bool Stm32Can::send_message(uint32_t tx_slot, const can_Message_t& message, on_sent_cb_t on_sent) {
    if (message.fd_frame || message.bit_rate_switching) {
        return false; // CAN FD not supported by hardware
    }
    if (message.len > 8) {
        return false;
    }

    // The mailboxes are always filled with the messages from those slots with
    // the highest priority messages.

    if (tx_slot >= kNTxSlots) {
        return false;
    }

    CRITICAL_SECTION() {
        // Set up new message
        slots_[tx_slot] = {true, message, on_sent};

        // If this `tx_slot` was already associated with a mailbox, abort that
        // mailbox.
        // The new message will be sent in the Abort (or Complete) ISR.
        if (mailbox_state_[0].slot_id == tx_slot) {
            return HAL_CAN_AbortTxRequest(&handle_, CAN_TX_MAILBOX0) == HAL_OK;
        } else if (mailbox_state_[1].slot_id == tx_slot) {
            return HAL_CAN_AbortTxRequest(&handle_, CAN_TX_MAILBOX1) == HAL_OK;
        } else if (mailbox_state_[2].slot_id == tx_slot) {
            return HAL_CAN_AbortTxRequest(&handle_, CAN_TX_MAILBOX2) == HAL_OK;

        // Else, if any mailbox is unused, send on that mailbox immediately
        } else if (mailbox_state_[0].slot_id == kSlotNone
                || mailbox_state_[1].slot_id == kSlotNone
                || mailbox_state_[2].slot_id == kSlotNone) {
            return send_now(tx_slot);

        // Else, find the mailbox with the lowest priority message and abort that
        // mailbox if it's message's priority is lower than the new messages's.
        // The new message will be sent in the Abort (or Complete) ISR.
        } else {
            int lowest_prio_mailbox_idx =
                mailbox_state_[0].msg_id >= mailbox_state_[1].msg_id &&
                mailbox_state_[0].msg_id >= mailbox_state_[2].msg_id ? 0 :
                mailbox_state_[1].msg_id >= mailbox_state_[2].msg_id ? 1 : 2;

            if (mailbox_state_[lowest_prio_mailbox_idx].msg_id > message.id) {
                return HAL_CAN_AbortTxRequest(&handle_, 1 << lowest_prio_mailbox_idx) == HAL_OK;
            }
        }
    }

    return true;
}

bool Stm32Can::subscribe(uint32_t rx_slot, const MsgIdFilterSpecs& filter, on_received_cb_t on_received, CanSubscription** handle) {
    auto it = std::find_if(subscriptions_.begin(), subscriptions_.end(), [](auto& subscription) {
        return subscription.fifo == kCanFifoNone;
    });

    if (it == subscriptions_.end()) {
        return false; // all subscription slots in use
    }

    if (rx_slot >= 2) {
        return false;
    }

    it->on_received = on_received;
    it->fifo = (rx_slot == 0) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    if (handle) {
        *handle = &*it;
    }

    bool is_extended = filter.id.index() == 1;
    uint32_t id = is_extended ?
                ((std::get<1>(filter.id) << 3) | (1 << 2)) :
                (std::get<0>(filter.id) << 21);
    uint32_t mask = (is_extended ? (filter.mask << 3) : (filter.mask << 21))
                | (1 << 2); // care about the is_extended bit

    CAN_FilterTypeDef hal_filter;
    hal_filter.FilterActivation = ENABLE;
    hal_filter.FilterBank = &*it - &subscriptions_[0];
    hal_filter.FilterFIFOAssignment = it->fifo;
    hal_filter.FilterIdHigh = (id >> 16) & 0xffff;
    hal_filter.FilterIdLow = id & 0xffff;
    hal_filter.FilterMaskIdHigh = (mask >> 16) & 0xffff;
    hal_filter.FilterMaskIdLow = mask & 0xffff;
    hal_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    hal_filter.FilterScale = CAN_FILTERSCALE_32BIT;

    if (HAL_CAN_ConfigFilter(&handle_, &hal_filter) != HAL_OK) {
        return false;
    }

    return true;
}

bool Stm32Can::unsubscribe(CanSubscription* handle) {
    Stm32CanSubscription* subscription = static_cast<Stm32CanSubscription*>(handle);
    if (subscription < subscriptions_.begin() || subscription >= subscriptions_.end()) {
        return false;
    }
    if (subscription->fifo != kCanFifoNone) {
        return false; // not in use
    }

    subscription->fifo = kCanFifoNone;

    CAN_FilterTypeDef hal_filter = {};
    hal_filter.FilterBank = (uint32_t)(subscription - &subscriptions_[0]);
    hal_filter.FilterActivation = DISABLE;
    return HAL_CAN_ConfigFilter(&handle_, &hal_filter) == HAL_OK;
}

bool Stm32Can::send_now(size_t slot_id) {
    can_Message_t& message = slots_[slot_id].message;

    CAN_TxHeaderTypeDef header;
    header.StdId = message.id;
    header.ExtId = message.id;
    header.IDE = message.is_extended_id ? CAN_ID_EXT : CAN_ID_STD;
    header.RTR = message.rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    header.DLC = message.len;
    header.TransmitGlobalTime = FunctionalState::DISABLE; // this param is ignored

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&handle_, &header, message.buf, &mailbox) != HAL_OK) {
        return false;
    }

    slots_[slot_id].waiting_for_mailbox = false;

    if (mailbox == CAN_TX_MAILBOX0) {
        mailbox_state_[0] = {slot_id, slots_[slot_id].message.id, slots_[slot_id].on_sent};
    } else if (mailbox == CAN_TX_MAILBOX1) {
        mailbox_state_[1] = {slot_id, slots_[slot_id].message.id, slots_[slot_id].on_sent};
    } else if (mailbox == CAN_TX_MAILBOX2) {
        mailbox_state_[2] = {slot_id, slots_[slot_id].message.id, slots_[slot_id].on_sent};
    }

    return true;
}

void Stm32Can::on_rx_fifo_pending(uint8_t fifo) {
    // This function runs on the application's event loop that was set in
    // start(). It works through the FIFO and finally re-enables interrupts for
    // that FIFO.

    while (HAL_CAN_GetRxFifoFillLevel(&handle_, fifo)) {
        CAN_RxHeaderTypeDef header;
        can_Message_t rxmsg;
        HAL_CAN_GetRxMessage(&handle_, fifo, &header, rxmsg.buf);

        rxmsg.is_extended_id = header.IDE;
        rxmsg.id = rxmsg.is_extended_id ? header.ExtId : header.StdId;  // If it's an extended message, pass the extended ID
        rxmsg.len = header.DLC;
        rxmsg.rtr = header.RTR;

        // TODO: this could be optimized with an ahead-of-time computed
        // index-to-filter map

        size_t fifo0_idx = 0;
        size_t fifo1_idx = 0;

        // Find the triggered subscription item based on header.FilterMatchIndex
        auto it = std::find_if(subscriptions_.begin(), subscriptions_.end(), [&](auto& s) {
            size_t current_idx = (s.fifo == CAN_RX_FIFO0 ? fifo0_idx : fifo1_idx)++;
            return (header.FilterMatchIndex == current_idx) && (s.fifo == fifo);
        });

        if (it == subscriptions_.end()) {
            continue;
        }

        it->on_received.invoke(rxmsg);
    }

    if (fifo == CAN_RX_FIFO0) {
        HAL_CAN_ActivateNotification(&handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
    } else {
        HAL_CAN_ActivateNotification(&handle_, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

void Stm32Can::on_tx_abort(uint32_t mailbox_idx) {
    // Reset associated slot to waiting state. This is valid whether or not the
    // slot already has a new message.
    slots_[mailbox_state_[mailbox_idx].slot_id].waiting_for_mailbox = true;

    mailbox_state_[mailbox_idx] = {};

    send_now();
}

void Stm32Can::on_tx_complete(uint32_t mailbox_idx) {
    // Clear the associated slot unless it was already filled with a new message
    if (!slots_[mailbox_state_[mailbox_idx].slot_id].waiting_for_mailbox) {
        slots_[mailbox_state_[mailbox_idx].slot_id] = {};
    }

    auto callback = mailbox_state_[mailbox_idx].on_sent;
    mailbox_state_[mailbox_idx] = {};

    send_now();

    callback.invoke(true);
}


void Stm32Can::send_now() {
    uint32_t lowest_id = UINT32_MAX;
    size_t lowest_id_slot = kSlotNone;

    // Out of all waiting slots, find the one with the lowest ID
    for (size_t i = 0; i < kNTxSlots; ++i) {
        if (slots_[i].waiting_for_mailbox) {
            if (lowest_id > slots_[i].message.id) {
                lowest_id = slots_[i].message.id;
                lowest_id_slot = i;
            }
        }
    }

    if (lowest_id_slot != kSlotNone) {
        send_now(lowest_id_slot); // TODO: log warning if this returns false
    }
}

void Stm32Can::on_error() {
    // TODO: differentiate between transient errors and errors that make the CAN interface go down
    on_error_.invoke(true);
}

Stm32Can& get_can(CAN_HandleTypeDef *hcan) {
    uintptr_t o = (uintptr_t)(&((Stm32Can*)nullptr)->handle_);
    return *reinterpret_cast<Stm32Can*>(reinterpret_cast<uintptr_t>(hcan) - o);
}


/* STM32 HAL callbacks -------------------------------------------------------*/

extern "C" {

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    // The notification will not refire immidiately so we don't need to disable it.
    //get_can(hcan).on_event_.invoke(MEMBER_CB(&get_can(hcan), on_tx_complete<0>));
    get_can(hcan).on_tx_complete(0);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    //get_can(hcan).on_event_.invoke(MEMBER_CB(&get_can(hcan), on_tx_complete<1>));
    get_can(hcan).on_tx_complete(1);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    //get_can(hcan).on_event_.invoke(MEMBER_CB(&get_can(hcan), on_tx_complete<2>));
    get_can(hcan).on_tx_complete(2);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {
    get_can(hcan).on_tx_abort(0);
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {
    get_can(hcan).on_tx_abort(1);
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {
    get_can(hcan).on_tx_abort(2);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    get_can(hcan).on_event_.invoke(MEMBER_CB(&get_can(hcan), on_rx_fifo_pending<CAN_RX_FIFO0>));
}

//void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
//    get_can(hcan).on_rx_fifo_full(0);
//}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    get_can(hcan).on_event_.invoke(MEMBER_CB(&get_can(hcan), on_rx_fifo_pending<CAN_RX_FIFO1>));
}

//void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
//    get_can(hcan).on_rx_fifo_full(1);
//}

void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) {
    // nothing to do
}

void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) {
    // nothing to do
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    get_can(hcan).on_event_.invoke(MEMBER_CB(&get_can(hcan), on_error));
}

}
