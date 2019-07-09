

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

enum InputMode_t {
    INPUT_MODE_INACTIVE,
    INPUT_MODE_PASSTHROUGH,
    INPUT_MODE_VEL_RAMP,
    INPUT_MODE_POS_FILTER,
    INPUT_MODE_MIX_CHANNELS,
    INPUT_MODE_TRAP_TRAJ,
};

// Fetch a specific signal from the message
template <typename T>
T can_getSignal(can_Message_t msg, const uint8_t startBit, const uint8_t length, const bool isIntel) {
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
    return retVal;
}

template<typename T>
float can_getSignal(can_Message_t msg, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    T retVal = can_getSignal<T>(msg, startBit, length, isIntel);
    return (retVal * factor) + offset;
}

template<typename T>
void can_setSignal(can_Message_t& msg, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel){
    uint64_t mask = (1ULL << length) - 1;
    uint64_t valAsBits = 0;
    std::memcpy(&valAsBits, &val, sizeof(T));
    

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
void can_setSignal(can_Message_t& msg, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    T scaledVal        = (val - offset) / factor;
    can_setSignal<T>(msg, scaledVal, startBit, length, isIntel);
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

    TEST_CASE("getSignal enums") {
        can_Message_t rxmsg;
        rxmsg.buf[0] = INPUT_MODE_MIX_CHANNELS;
        rxmsg.buf[1] = INPUT_MODE_PASSTHROUGH;
        CHECK(static_cast<InputMode_t>(can_getSignal<InputMode_t>(rxmsg, 0, 8, true, 1, 0)) == INPUT_MODE_MIX_CHANNELS);
        CHECK(static_cast<InputMode_t>(can_getSignal<InputMode_t>(rxmsg, 8, 8, true, 1, 0)) == INPUT_MODE_PASSTHROUGH);
    }
}


TEST_SUITE("delta_enc"){
    // Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
    int mod(int dividend, int divisor){
        int r = dividend % divisor;
        return (r < 0) ? (r + divisor) : r;
    }

    int getDelta(int pos_abs, int count_in_cpr, int cpr) {
        int delta_enc = pos_abs - count_in_cpr;
        delta_enc = mod(delta_enc, cpr);
        if (delta_enc > (cpr / 2))
            delta_enc -= cpr;
        return delta_enc;
    }

    TEST_CASE("mod"){

        int cpr = 1000;

        // Check moves around 0
        CHECK(getDelta(1, 0, cpr) == 1);
        CHECK(getDelta(0, 1, cpr) == -1);
        CHECK(getDelta(999, 0, cpr) == -1);
        CHECK(getDelta(50, 650, cpr) == 400);
        CHECK(getDelta(650, 50, cpr) == -400);
        CHECK(getDelta(50, 500, cpr) == -450);
        CHECK(getDelta(500, 50, cpr) == 450);

        
        // Test moving a distance larger than cpr / 2
        CHECK(getDelta(950, 450, cpr) == 500);
        CHECK(getDelta(451, 950, cpr) == -499);
        CHECK(getDelta(450, 950, cpr) == 500);
        
        // Test handling around mid-point
        CHECK(getDelta(501, 499, cpr) == 2);   
        CHECK(getDelta(499, 501, cpr) == -2);
        CHECK(getDelta(550, 450, cpr) == 100);
        CHECK(getDelta(450, 550, cpr) == -100);
    }
}

TEST_SUITE("velLimiter") {
// Velocity limiting in current mode
#include <algorithm>
using doctest::Approx;

    auto limitVel(float vel_limit, float vel_estimate, float vel_gain, float Iq) {
        float Imax = (vel_limit - vel_estimate) * vel_gain;
        float Imin = (-vel_limit - vel_estimate) * vel_gain;
        return std::clamp(Iq, Imin, Imax);
    }

    TEST_CASE("limit Vel") {
        CHECK(limitVel(0, 0, 0, 0) == 0.0f);
        CHECK(limitVel(1000.0f, 1.0f, 0.0f, 0.0f) == 0.0f);
        CHECK(limitVel(1000.0f, 500.0f, 1.0f, 1.0f) == 1.0f);
        CHECK(limitVel(1000.0f, 500.0f, 1.0f, -20.0f) == -20.0f);
        CHECK(limitVel(1000.0f, 999.0f, 1.0f, 2.0f) == 1.0f);
        CHECK(limitVel(1000.0f, 999.0f, 1.0f, -5.0f) == -5.0f);
        CHECK(limitVel(1000.0f, -999.0f, 1.0f, -5.0f) == -1.0f);
        CHECK(limitVel(1000.0f, -999.0f, 1.0f, 5.0f) == 5.0f);
        CHECK(limitVel(1000.0f, 0.0f, 1.0f, 1.0f) == 1.0f);
        CHECK(limitVel(1000.0f, 0.0f, 1.0f, -1.0f) == -1.0f);
    }

    TEST_CASE("Accelerating"){
        CHECK(limitVel(200000.0f, 195000.0f, 5.0E-4f, 30.0f) == 2.5f);
        CHECK(limitVel(200000.0f, 205000.0f, 5.0E-4f, 30.0f) == -2.5f);
        CHECK(limitVel(200000.0f, -195000.0f, 5.0E-4, -30.0f) == -2.5f);
        CHECK(limitVel(200000.0f, -205000.0f, 5.0E-4f, -30.0f) == 2.5f);
    }

    TEST_CASE("Decelerating"){
        CHECK(limitVel(200000.0f, 195000.0f, 5.0E-4f, -30.0f) == -30.0f);
        CHECK(limitVel(200000.0f, 205000.0f, 5.0E-4f, -30.0f) == -30.0f);
        CHECK(limitVel(200000.0f, -195000.0f, 5.0E-4, 30.0f) == 30.0f);
        CHECK(limitVel(200000.0f, -205000.0f, 5.0E-4f, 30.0f) == 30.0f);
    }

    TEST_CASE("Over-Center"){
        CHECK(limitVel(20000.0f, 1000.0f, 5.0E-4f, 30.0f) == 9.5f);
        CHECK(limitVel(20000.0f, -1000.0f, 5.0E-4f, 30.0f) == Approx(10.5f));
    }
}

TEST_SUITE("vel_ramp") {
    float vel_ramp_old(float input_vel_, float vel_setpoint_, float vel_ramp_rate) {
        float max_step_size = 0.000125f * vel_ramp_rate;
        float full_step     = input_vel_ - vel_setpoint_;
        float step;
        if (fabsf(full_step) > max_step_size) {
            step = std::copysignf(max_step_size, full_step);
        } else {
            step = full_step;
        }
        return step;
    }

    float vel_ramp_new(float input_vel_, float vel_setpoint_, float vel_ramp_rate){
        float max_step_size = 0.000125f * vel_ramp_rate;
        float full_step     = input_vel_ - vel_setpoint_;
        return std::clamp(full_step, -max_step_size, max_step_size);
    }

    TEST_CASE("Blah") {
        float vel_setpoint = 0.0f;
        float vel_ramp_rate = 8000;
        float input_vel = 0.0f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = 10.0f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = 10000.0f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = -10000.0f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = -0.1234f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = 0.1234f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));
    }
}