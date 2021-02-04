#ifndef __STM32_CAN_HPP
#define __STM32_CAN_HPP

#include <interfaces/canbus.hpp>
#include "stm32_system.h"

class Stm32Can : public CanBusBase {
public:
    Stm32Can(CAN_TypeDef* instance) : handle_{.Instance = instance} {}

    IRQn_Type get_tx_irqn() {
        switch ((uint32_t)handle_.Instance) {
            case CAN1_BASE: return CAN1_TX_IRQn;
            default: return UsageFault_IRQn;
        }
    }

    IRQn_Type get_rx0_irqn() {
        switch ((uint32_t)handle_.Instance) {
            case CAN1_BASE: return CAN1_RX0_IRQn;
            default: return UsageFault_IRQn;
        }
    }

    IRQn_Type get_rx1_irqn() {
        switch ((uint32_t)handle_.Instance) {
            case CAN1_BASE: return CAN1_RX1_IRQn;
            default: return UsageFault_IRQn;
        }
    }

    IRQn_Type get_sce_irqn() {
        switch ((uint32_t)handle_.Instance) {
            case CAN1_BASE: return CAN1_SCE_IRQn;
            default: return UsageFault_IRQn;
        }
    }

    bool init(CAN_InitTypeDef config, uint32_t can_freq);

    // Callbacks called from interrupt routines
    template<uint8_t fifo> void on_rx_fifo_pending() { return on_rx_fifo_pending(fifo); }
    void on_tx_abort(uint32_t mailbox);
    void on_tx_complete(uint32_t mailbox);
    void on_error();

    CAN_HandleTypeDef handle_;
    on_event_cb_t on_event_;

private:
    static const uint8_t kCanFifoNone = 0xff;
    static const size_t kNTxSlots = 10;
    static const size_t kSlotNone = SIZE_MAX;
    static const uint32_t CAN_TX_MAILBOX_NONE = UINT32_MAX;
    static const uint32_t CAN_TX_MAILBOX_WAITING = UINT32_MAX - 1;

    struct Stm32CanSubscription : CanSubscription {
        uint8_t fifo = kCanFifoNone; // kCanFifoNone means the slot is not in use
        on_received_cb_t on_received;
    };

    struct Slot {
        bool waiting_for_mailbox = false;
        can_Message_t message;
        on_sent_cb_t on_sent;
    };

    struct MailboxState {
        uint32_t slot_id = kSlotNone;
        uint32_t msg_id;
        on_sent_cb_t on_sent;
    };

    bool is_valid_baud_rate(uint32_t nominal_baud_rate, uint32_t data_baud_rate) final;
    bool start(uint32_t nominal_baud_rate, uint32_t data_baud_rate, on_event_cb_t on_event, on_error_cb_t on_error) final;
    bool stop() final;
    bool send_message(uint32_t tx_slot, const can_Message_t& message, on_sent_cb_t on_sent) final;
    bool subscribe(uint32_t rx_slot, const MsgIdFilterSpecs& filter, on_received_cb_t on_received, CanSubscription** handle) final;
    bool unsubscribe(CanSubscription* handle) final;

    //bool enable_subscription(Stm32CanSubscription* handle);
    bool send_now(size_t slot_id);
    void send_now();

    void on_rx_fifo_pending(uint8_t fifo);

    uint32_t can_freq_;

    // Hardware supports at most 28 filters unless we do optimizations. For now
    // we don't need that many.
    std::array<Stm32CanSubscription, 8> subscriptions_;

    on_error_cb_t on_error_;

    std::array<Slot, kNTxSlots> slots_;
    std::array<MailboxState, 3> mailbox_state_;
};

#define DEFINE_STM32_CAN(name, instance) \
    Stm32Can name{instance}; \
    extern "C" void instance ## _TX_IRQHandler(void) { \
        COUNT_IRQ(instance ## _TX_IRQn); \
        HAL_CAN_IRQHandler(&name.handle_); \
    } \
    extern "C" void instance ## _RX0_IRQHandler(void) { \
        COUNT_IRQ(instance ## _RX0_IRQn); \
        HAL_CAN_IRQHandler(&name.handle_); \
    } \
    extern "C" void instance ## _RX1_IRQHandler(void) { \
        COUNT_IRQ(instance ## _RX1_IRQn); \
        HAL_CAN_IRQHandler(&name.handle_); \
    } \
    extern "C" void instance ## _SCE_IRQHandler(void) { \
        COUNT_IRQ(instance ## _SCE_IRQn); \
        HAL_CAN_IRQHandler(&name.handle_); \
    }

#endif  // __STM32_CAN_HPP
