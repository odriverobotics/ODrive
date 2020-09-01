#pragma once

#include <stdint.h>
#include <algorithm>
#include <cstring>
#include <iterator>

struct can_Message_t {
    uint32_t id = 0x000;  // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF
    bool isExt = false;
    bool rtr = false;
    uint8_t len = 8;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
} ;

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

#include <iterator>
template <typename T>
constexpr T can_getSignal(can_Message_t msg, const uint8_t startBit, const uint8_t length, const bool isIntel) {
    uint64_t tempVal = 0;
    uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;

    if (isIntel) {
        std::memcpy(&tempVal, msg.buf, sizeof(tempVal));
        tempVal = (tempVal >> startBit) & mask;
    } else {
        std::reverse(std::begin(msg.buf), std::end(msg.buf));
        std::memcpy(&tempVal, msg.buf, sizeof(tempVal));
        tempVal = (tempVal >> (64 - startBit - length)) & mask;
    }

    T retVal;
    std::memcpy(&retVal, &tempVal, sizeof(T));
    return retVal;
}

template <typename T>
constexpr void can_setSignal(can_Message_t& msg, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel) {
    uint64_t valAsBits = 0;
    std::memcpy(&valAsBits, &val, sizeof(val));

    uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;

    if (isIntel) {
        uint64_t data = 0;
        std::memcpy(&data, msg.buf, sizeof(data));

        data &= ~(mask << startBit);
        data |= valAsBits << startBit;

        std::memcpy(msg.buf, &data, sizeof(data));
    } else {
        uint64_t data = 0;
        std::reverse(std::begin(msg.buf), std::end(msg.buf));
        std::memcpy(&data, msg.buf, sizeof(data));

        data &= ~(mask << (64 - startBit - length));
        data |= valAsBits << (64 - startBit - length);

        std::memcpy(msg.buf, &data, sizeof(data));
        std::reverse(std::begin(msg.buf), std::end(msg.buf));
    }
}

template<typename T>
void can_setSignal(can_Message_t& msg, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    T scaledVal = static_cast<T>((val - offset) / factor);
    can_setSignal<T>(msg, scaledVal, startBit, length, isIntel);
}

template<typename T>
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