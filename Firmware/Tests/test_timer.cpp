#define DOCTEST_IMPLEMENT
#include <doctest.h>
#include "MotorControl/timer.hpp"

TEST_CASE("Timer"){
    Timer<int> myTimer;
    myTimer.setTimeout(10);
    myTimer.setInterval(1);
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