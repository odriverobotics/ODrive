#ifndef __CANBUS_HPP
#define __CANBUS_HPP

#include "can_helpers.hpp"
#include <variant>
#include <fibre/callback.hpp>

struct MsgIdFilterSpecs {
    std::variant<uint16_t, uint32_t> id;
    uint32_t mask;
};

class CanBusBase {
public:
    using on_event_cb_t = fibre::Callback<void, fibre::Callback<void>>;
    using on_error_cb_t = fibre::Callback<void, bool>;
    using on_sent_cb_t = fibre::Callback<void, bool>;
    using on_received_cb_t = fibre::Callback<void, const can_Message_t&>;

    struct CanSubscription {};

    /**
     * @brief Checks if the specified baud rate combination is compatible with
     * this interface.
     * 
     * This function can be used regardless of started/stopped state of the CAN
     * bus.
     * For interfaces that don't support CAN FD, the function returns false if
     * the two baud rates mismatch.
     * 
     * @param nominal_baud_rate: The CAN message header baud rate in bits per
     *        second.
     * @param data_baud_rate: The payload baud rate in bits per second.
     */
    virtual bool is_valid_baud_rate(uint32_t nominal_baud_rate, uint32_t data_baud_rate) = 0;

    /**
     * @brief Brings the CAN bus interface up.
     * 
     * When the CAN bus is up (and only then), send_message() can be called and
     * the subscriptions get notified on corresponding incoming messages.
     * 
     * @param nominal_baud_rate: The CAN message header baud rate in bits per
     *        second.
     * @param data_baud_rate: The payload baud rate in bits per second.
     * @param rx_event_loop: This callback is used to put event tasks on the
     *        caller's event loop. The callback can be called in interrupt
     *        context. See also `subscribe()`.
     * @param on_error: called when an error condition occurs. A bool argument
     *        is passed to indicate if the error is permanent and the CAN bus is down.
     * 
     * @returns: True if the CAN bus was started, false otherwise. A possible
     *           reason for a failed start is an incompatible baud rate.
     */
    virtual bool start(uint32_t nominal_baud_rate, uint32_t data_baud_rate, on_event_cb_t rx_event_loop, on_error_cb_t on_error) = 0;

    /**
     * @brief Stops the CAN bus interface.
     */
    virtual bool stop() = 0;

    /**
     * @brief Sends the specified CAN message.
     * 
     * @returns: true on success or false otherwise (e.g. if the send queue is
     * full).
     */
    virtual bool send_message(uint32_t tx_slot, const can_Message_t& message, on_sent_cb_t on_sent) = 0;

    /**
     * @brief Registers a callback that will be invoked for every incoming CAN
     * message that matches the filter.
     * 
     * This function can be used regardless of started/stopped state of the CAN
     * bus.
     * 
     * @param on_received: Called when a matching message arrives. This is executed
     * @param handle: On success this handle is set to an opaque pointer that
     *        can be used to cancel the subscription.
     * 
     * @returns: true on success or false otherwise (e.g. if the maximum number
     * of subscriptions has been reached).
     */
    virtual bool subscribe(uint32_t rx_slot, const MsgIdFilterSpecs& filter, on_received_cb_t on_received, CanSubscription** handle) = 0;

    /**
     * @brief Deregisters a callback that was previously registered with subscribe().
     */
    virtual bool unsubscribe(CanSubscription* handle) = 0;
};

#endif // __CANBUS_HPP