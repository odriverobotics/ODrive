#ifndef __CANBUS_HPP
#define __CANBUS_HPP

#include "can_helpers.hpp"
#include <variant>

struct MsgIdFilterSpecs {
    std::variant<uint16_t, uint32_t> id;
    uint32_t mask;
};

class CanBusBase {
public:
    typedef void(*on_can_message_cb_t)(void* ctx, const can_Message_t& message);
    struct CanSubscription {};

    /**
     * @brief Sends the specified CAN message.
     * 
     * @returns: true on success or false otherwise (e.g. if the send queue is
     * full).
     */
    virtual bool send_message(const can_Message_t& message) = 0;

    /**
     * @brief Registers a callback that will be invoked for every incoming CAN
     * message that matches the filter.
     * 
     * @param handle: On success this handle is set to an opaque pointer that
     *        can be used to cancel the subscription.
     * 
     * @returns: true on success or false otherwise (e.g. if the maximum number
     * of subscriptions has been reached).
     */
    virtual bool subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) = 0;

    /**
     * @brief Deregisters a callback that was previously registered with subscribe().
     */
    virtual bool unsubscribe(CanSubscription* handle) = 0;
};

#endif // __CANBUS_HPP