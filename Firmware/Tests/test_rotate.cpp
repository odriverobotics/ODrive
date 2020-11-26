/* MIT License

Copyright (c) 2016-2021 ODrive Robotics, Inc

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

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