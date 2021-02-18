#pragma once

#include <stdint.h>

#include <algorithm>
#include <cstring>
#include <iterator>

enum CanFrameType {
    kCanFrameTypeClassical,
    kCanFrameTypeClassicalRtr,
};

struct can_Message_t {
    uint32_t id = 0x000;  // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF

    /**
     * Controls the IDE bit.
     */
    bool is_extended_id = false;

    /**
     * Remote Transmission Request. Controls the RTR bit in a Classical CAN
     * message. Must be false if `fd_frame` is true. 
     */
    bool rtr = false;

    /**
     * Controls the BRS bit in a CAN FD frame. If true, the payload and part of
     * the header/footer are transmitted at `data_baud_rate` instead of
     * `nominal_baud_rate`. Must be false if `fd_frame` is false.
     */
    bool bit_rate_switching = false;

    /**
     * Controls the FDF bit (aka r0 in Classical CAN). Must be false on
     * interfaces that don't support CAN FD.
     */
    bool fd_frame = false;

    uint8_t len = 8;
    uint8_t buf[64] = {0};
};

struct can_Signal_t {
    const uint8_t startBit;
    const uint8_t length;
    const bool isIntel;
    const float factor;
    const float offset;
};

struct can_Cyclic_t {
    uint32_t cycleTime_ms;
    uint32_t lastTime_ms;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"  // Make sure to check these functions on your system
template <typename T>
constexpr T can_getSignal(const can_Message_t& msg, const uint8_t startBit, const uint8_t length, const bool isIntel) {
    uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;

    uint64_t tempVal = *(reinterpret_cast<const uint64_t*>(&msg.buf[startBit / 8]));
    if (isIntel) {
        tempVal = (tempVal >> startBit % 8) & mask;
    } else {
        tempVal = __builtin_bswap64(tempVal);
        tempVal = (tempVal >> (64 - (startBit % 8) - length)) & mask;
    }

    return *(reinterpret_cast<T*>(&tempVal));
}

template <typename T>
constexpr void can_setSignal(can_Message_t& msg, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel) {
    union aliastype {
        aliastype() : valAsBits(0) {}
        T tempVal;
        uint64_t valAsBits;
    };

    const uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;
    const uint8_t shift = isIntel ? (startBit % 8) : (64 - startBit % 8) - length;

    aliastype valAlias;
    valAlias.tempVal = val;
    valAlias.valAsBits &= mask;

    uint64_t data = *(reinterpret_cast<const uint64_t*>(&msg.buf[startBit / 8]));
    if (isIntel) {
        data &= ~(mask << shift);
        data |= valAlias.valAsBits << shift;
    } else {
        data = __builtin_bswap64(data);
        data &= ~(mask << shift);
        data |= valAlias.valAsBits << shift;
        data = __builtin_bswap64(data);
    }
    *(reinterpret_cast<uint64_t*>(&msg.buf[startBit / 8])) = data;
}
#pragma GCC diagnostic pop

template <typename T>
void can_setSignal(can_Message_t& msg, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    T scaledVal = static_cast<T>((val - offset) / factor);
    can_setSignal<T>(msg, scaledVal, startBit, length, isIntel);
}

template <typename T>
float can_getSignal(can_Message_t msg, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    T retVal = can_getSignal<T>(msg, startBit, length, isIntel);
    return (retVal * factor) + offset;
}

template <typename T>
float can_getSignal(can_Message_t msg, const can_Signal_t& signal) {
    return can_getSignal<T>(msg, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
}

template <typename T>
void can_setSignal(can_Message_t& msg, const T& val, const can_Signal_t& signal) {
    can_setSignal(msg, val, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
}