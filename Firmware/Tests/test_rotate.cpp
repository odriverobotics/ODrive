#include <doctest.h>

#include <algorithm>
#include <array>
#include <iostream>

TEST_CASE("Rotate Axis State"){
    std::array<int, 10> testArr = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    const int& currentVal = testArr.front();
    CHECK(currentVal == testArr[0]);

    std::rotate(testArr.begin(), testArr.begin() + 1, testArr.end());
    CHECK(currentVal == testArr[0]);
    CHECK(currentVal == 1);

    CHECK(testArr.back() == 0);

    std::rotate(testArr.begin(), testArr.begin() + 1, testArr.end());
    CHECK(currentVal == testArr[0]);
    CHECK(currentVal == 2);
    CHECK(testArr.back() == 1);


}

TEST_CASE("Fill Test"){
    std::array<int, 10> testArr = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::fill(testArr.begin(), testArr.end(), 6);
    for(const auto& val : testArr){
        CHECK(val == 6);
    }
}