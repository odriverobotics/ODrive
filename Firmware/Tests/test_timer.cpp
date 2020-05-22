#include <doctest.h>
#include "MotorControl/timer.hpp"
#include <stdint.h>

TEST_CASE_TEMPLATE("Timer2", T, float, int, char, uint32_t){
    Timer<T> myTimer;
    myTimer.setTimeout(10);
    myTimer.setIncrement(1);
    CHECK(!myTimer.expired());

    myTimer.start();
    CHECK(!myTimer.expired());
    for(int i = 0; i < 9; ++i){
        myTimer.update();
        CHECK(!myTimer.expired());
    }

    myTimer.update();
    CHECK(myTimer.expired());

    myTimer.stop();
    CHECK(myTimer.expired());
    
    myTimer.start();
    CHECK(myTimer.expired());

    myTimer.reset();
    CHECK(!myTimer.expired());
}