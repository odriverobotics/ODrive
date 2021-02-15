
#include <doctest.h>
#include <algorithm>
#include <cstring>

#include "communication/can/can_helpers.hpp"

enum InputMode {
    INPUT_MODE_INACTIVE,
    INPUT_MODE_PASSTHROUGH,
    INPUT_MODE_VEL_RAMP,
    INPUT_MODE_POS_FILTER,
    INPUT_MODE_MIX_CHANNELS,
    INPUT_MODE_TRAP_TRAJ,
};

TEST_SUITE("CAN Functions") {
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

        val = can_getSignal<uint16_t>(rxmsg, 0, 16, true, 1, 0);
        CHECK(val == 0x1234);

        val = can_getSignal<uint16_t>(rxmsg, 0, 16, true);
        CHECK(val == 0x1234);

        val = can_getSignal<uint16_t>(rxmsg, 0, 16, false, 1, 0);
        CHECK(val == 0x3412);

        float myFloat = 1234.6789f;
        std::memcpy(rxmsg.buf, &myFloat, sizeof(myFloat));
        auto floatVal = can_getSignal<float>(rxmsg, 0, 32, true, 1, 0);
        CHECK(floatVal == 1234.6789f);

        can_Message_t msg;
        msg.id = 0x00E;
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

    TEST_CASE("getSignal enums") {
        can_Message_t rxmsg;
        rxmsg.buf[0] = INPUT_MODE_MIX_CHANNELS;
        rxmsg.buf[1] = INPUT_MODE_PASSTHROUGH;
        CHECK(static_cast<InputMode>(can_getSignal<InputMode>(rxmsg, 0, 8, true, 1, 0)) == INPUT_MODE_MIX_CHANNELS);
        CHECK(static_cast<InputMode>(can_getSignal<InputMode>(rxmsg, 8, 8, true, 1, 0)) == INPUT_MODE_PASSTHROUGH);
    }
}