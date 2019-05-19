

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
    uint32_t id = 0x000;  // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF
    bool isExt = false;
    bool rtr = false;
    uint8_t len = 8;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

struct can_Signal_t {
    uint8_t startBit = 0;
    uint8_t length = 16;
    bool isIntel = true;
    float factor = 1.0f;
    float offset = 0.0f;
};

// Fetch a specific signal from the message
template <typename T>
T getSignal(can_Message_t msg, uint8_t startBit, uint8_t length, bool isIntel, float factor, float offset) {
    uint64_t tempVal = 0;
    uint64_t mask = (1ULL << length) - 1;

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
    return static_cast<T>((retVal * factor) + offset);
}

// template<typename T>
// T can_getSignal(can_Message_t msg, can_Signal_t signal){
//     return can_getSignal(msg, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
// }

TEST_SUITE("getSignal") {
    TEST_CASE("reverse") {
        can_Message_t rxmsg;
        rxmsg.id = 0x000;
        rxmsg.isExt = false;
        rxmsg.len = 8;

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

        val = getSignal<uint16_t>(rxmsg, 0, 16, true, 1, 0);
        CHECK(val == 0x1234);

        val = getSignal<uint16_t>(rxmsg, 0, 16, false, 1, 0);
        CHECK(val == 0x3412);

        float myFloat = 1234.6789f;
        std::memcpy(rxmsg.buf, &myFloat, sizeof(myFloat));
        auto floatVal = getSignal<float>(rxmsg, 0, 32, true, 1, 0);
        CHECK(floatVal == 1234.6789f);
    }
}