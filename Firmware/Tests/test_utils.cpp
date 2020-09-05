#include <doctest.h>

#include <limits>
#include <utils.hpp>

namespace old {
// Old fmodf_pos
inline float fmodf_pos(float x, float y) {
    float out = fmodf(x, y);
    if (out < 0.0f)
        out += y;
    return out;
}

/**
 * @brief Similar to modulo operator, except that the output range is centered
 * around zero.
 * The returned value is always in the range [-pm_range, pm_range).
 */
inline float wrap_pm(float x, float pm_range) {
    return fmodf_pos(x + pm_range, 2.0f * pm_range) - pm_range;
}

inline float wrap_pm_pi(float theta) {
    return wrap_pm(theta, M_PI);
}

};  // namespace old

TEST_SUITE("fmod") {
    constexpr auto eps = std::numeric_limits<float>::epsilon();

    inline float fmodTest(float x, float y) {
        auto oldMod = old::fmodf_pos(x, y);
        auto newMod = fmodf_pos(x, y);

        CHECK(oldMod == doctest::Approx(newMod).epsilon(8 * eps));
    }
    
    TEST_CASE("fmodf_test") {
        fmodTest(10.0f, 1.0f);
        fmodTest(1.1f, 0.1f);
        fmodTest(0.1f, 1.0f);
    }
}