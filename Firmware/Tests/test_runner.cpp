

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define DOCTEST_CONFIG_TREAT_CHAR_STAR_AS_STRING
#define DOCTEST_CONFIG_USE_STD_HEADERS
#define DOCTEST_CONFIG_NO_TRY_CATCH_IN_ASSERTS
#define DOCTEST_CONFIG_NO_EXCEPTIONS
#define DOCTEST_CONFIG_NO_WINDOWS_SEH
#define DOCTEST_CONFIG_NO_POSIX_SIGNALS
// #define DOCTEST_CONFIG_VOID_CAST_EXPRESSIONS

#include <doctest.h>

using std::cout;
using std::endl;

struct can_Message_t {
    uint32_t id    = 0x000;  // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF
    bool isExt     = false;
    bool rtr       = false;
    uint8_t len    = 8;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

struct can_Signal_t {
    const uint8_t startBit;
    const uint8_t length;
    const bool isIntel;
    const float factor;
    const float offset;
};

// Fetch a specific signal from the message
template <typename T>
float can_getSignal(can_Message_t msg, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    uint64_t tempVal = 0;
    uint64_t mask    = (1ULL << length) - 1;

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
    return (retVal * factor) + offset;
}

template <typename T>
void can_setSignal(can_Message_t& msg, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    T scaledVal        = (val - offset) / factor;
    uint64_t valAsBits = 0;
    std::memcpy(&valAsBits, &scaledVal, sizeof(scaledVal));

    uint64_t mask = (1ULL << length) - 1;

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

template <typename T>
float can_getSignal(can_Message_t msg, const can_Signal_t& signal) {
    return can_getSignal<T>(msg, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
}

template <typename T>
void can_setSignal(can_Message_t& msg, const T& val, const can_Signal_t& signal) {
    can_setSignal(msg, val, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
}

TEST_CASE("fake") {
    cout << endl;
}

TEST_SUITE("CAN Functions") {
    TEST_CASE("reverse") {
        can_Message_t rxmsg;
        rxmsg.id    = 0x000;
        rxmsg.isExt = false;
        rxmsg.len   = 8;

        rxmsg.buf[0] = 0x12;
        rxmsg.buf[1] = 0x34;

        std::reverse(std::begin(rxmsg.buf), std::end(rxmsg.buf));
        CHECK(rxmsg.buf[0] == 0x00);
        CHECK(rxmsg.buf[6] == 0x34);
        CHECK(rxmsg.buf[7] == 0x12);
    }

    TEST_CASE("getSignal") {
        can_Message_t rxmsg;

        auto val = 0x1234;
        std::memcpy(rxmsg.buf, &val, sizeof(val));

        val = can_getSignal<uint16_t>(rxmsg, 0, 16, true, 1, 0);
        CHECK(val == 0x1234);

        val = can_getSignal<uint16_t>(rxmsg, 0, 16, false, 1, 0);
        CHECK(val == 0x3412);

        float myFloat = 1234.6789f;
        std::memcpy(rxmsg.buf, &myFloat, sizeof(myFloat));
        auto floatVal = can_getSignal<float>(rxmsg, 0, 32, true, 1, 0);
        CHECK(floatVal == 1234.6789f);

        can_Message_t msg;
        msg.id     = 0x00E;
        msg.buf[0] = 0x96;
        msg.buf[1] = 0x00;
        msg.buf[2] = 0x00;
        msg.buf[3] = 0x00;
        CHECK(can_getSignal<int32_t>(msg, 0, 32, true, 0.01f, 0.0f) == 1.50f);
    }

    TEST_CASE("setSignal") {
        can_Message_t txmsg;

        can_setSignal<uint16_t>(txmsg, 0x1234, 0, 16, true, 1.0f, 0.0f);
        CHECK(can_getSignal<uint16_t>(txmsg, 0, 16, true, 1.0f, 0.0f) == 0x1234);

        can_setSignal<uint16_t>(txmsg, 0xABCD, 16, 16, true, 1.0f, 0.0f);
        CHECK(can_getSignal<uint16_t>(txmsg, 0, 16, true, 1.0f, 0.0f) == 0x1234);
        CHECK(can_getSignal<uint16_t>(txmsg, 16, 16, true, 1.0f, 0.0f) == 0xABCD);

        can_setSignal<float>(txmsg, 1234.5678f, 32, 32, true, 1.0f, 0.0f);
        CHECK(can_getSignal<uint16_t>(txmsg, 0, 16, true, 1.0f, 0.0f) == 0x1234);
        CHECK(can_getSignal<uint16_t>(txmsg, 16, 16, true, 1.0f, 0.0f) == 0xABCD);
        CHECK(can_getSignal<float>(txmsg, 32, 32, true, 1.0f, 0.0f));

        can_setSignal<uint16_t>(txmsg, 0x1234, 0, 16, false, 1.0f, 0.0f);
        CHECK(can_getSignal<uint16_t>(txmsg, 0, 16, false, 1.0f, 0.0f) == 0x1234);
        CHECK(can_getSignal<uint16_t>(txmsg, 16, 16, true, 1.0f, 0.0f) == 0xABCD);
        CHECK(can_getSignal<float>(txmsg, 32, 32, true, 1.0f, 0.0f));

        can_setSignal<float>(txmsg, 234981.0f, 12, 32, false, 2.0f, 1.1f);
        CHECK(can_getSignal<float>(txmsg, 12, 32, false, 2.0f, 1.1f) == 234981.0f);
    }
}