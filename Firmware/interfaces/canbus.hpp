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
     * @param nominal_baud_rate: The baud rate that is used for the arbitration
     *        phase, or, if bit rate switching is not used, the whole message.
     * @param data_baud_rate: The baud rate that is used for the payload of a
     *        CAN FD message that uses bit rate switching.
     */
    virtual bool is_valid_baud_rate(uint32_t nominal_baud_rate, uint32_t data_baud_rate) = 0;

    /**
     * @brief Brings the CAN bus interface up.
     * 
     * When the CAN bus is up (and only then), send_message() can be called and
     * the subscriptions get notified on corresponding incoming messages.
     * 
     * @param nominal_baud_rate: The baud rate that is used for the arbitration
     *        phase, or, if bit rate switching is not used, the whole message.
     * @param data_baud_rate: The baud rate that is used for the payload of
     *        CAN FD messages that use bit rate switching.
     * @param rx_event_loop: This callback is used to put event tasks on the
     *        caller's event loop. The callback can be called in interrupt
     *        context. See also `subscribe()`.
     * @param on_error: called when an error condition occurs. A bool argument
     *        is passed to indicate if the error is permanent and the CAN bus is
     *        down.
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
     * @param tx_slot: The TX slot into which to place this message. If an
     *        earlier message is still pending in this slot it will be evicted
     *        by the new message.
     *        The number of available TX slots is implementation specific.
     * @param message: The message to send.
     * @param on_sent: A callback that is invoked when the message was
     *        successfully sent. Can be invoked in an interrupt context.
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
     * If several overlapping filters are registered, one or more of them may
     * be called, depending on the implementation.
     * 
     * @param rx_slot: Identifies the RX FIFO or buffer into which messages
     *        matching this filter should be placed.
     *        The number of available RX slots is implementation specific and
     *        can be as low as one.
     * @param filter: Specifies which messages to accept on this subscription.
     * @param on_received: Called when a matching message arrives. This is
     *        executed in the execution context given by `rx_event_loop`.
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