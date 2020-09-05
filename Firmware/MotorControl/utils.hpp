#pragma once

#include <stdint.h>
#include <limits>
#include <algorithm>
#include <array>

/**
 * @brief Flash size register address
 */
#define ID_FLASH_ADDRESS (0x1FFF7A22)

/**
 * @brief Device ID register address
 */
#define ID_DBGMCU_IDCODE (0xE0042000)

/**
 * "Returns" the device signature
 *
 * Possible returns:
 *    - 0x0413: STM32F405xx/07xx and STM32F415xx/17xx)
 *    - 0x0419: STM32F42xxx and STM32F43xxx
 *    - 0x0423: STM32F401xB/C
 *    - 0x0433: STM32F401xD/E
 *    - 0x0431: STM32F411xC/E
 *
 * Returned data is in 16-bit mode, but only bits 11:0 are valid, bits 15:12 are always 0.
 * Defined as macro
 */
#define STM_ID_GetSignature() ((*(uint16_t *)(ID_DBGMCU_IDCODE)) & 0x0FFF)

/**
 * "Returns" the device revision
 *
 * Revisions possible:
 *    - 0x1000: Revision A
 *    - 0x1001: Revision Z
 *    - 0x1003: Revision Y
 *    - 0x1007: Revision 1
 *    - 0x2001: Revision 3
 *
 * Returned data is in 16-bit mode.
 */
#define STM_ID_GetRevision() (*(uint16_t *)(ID_DBGMCU_IDCODE + 2))

/**
* "Returns" the Flash size
*
* Returned data is in 16-bit mode, returned value is flash size in kB (kilo bytes).
*/
#define STM_ID_GetFlashSize() (*(uint16_t *)(ID_FLASH_ADDRESS))

#ifdef M_PI
#undef M_PI
#endif

// Math Constants
constexpr float M_PI = 3.14159265358979323846f;
constexpr float one_by_sqrt3 = 0.57735026919f;
constexpr float two_by_sqrt3 = 1.15470053838f;
constexpr float sqrt3_by_2 = 0.86602540378f;

template<typename T>
constexpr T SQ(const T& x){
    return x * x;
}

/**
 * @brief Small helper to make array with known size
 * in contrast to initializer lists the number of arguments
 * has to match exactly. Whereas initializer lists allow
 * less arguments.
 */
template <class T, class... Tail>
std::array<T, 1 + sizeof...(Tail)> make_array(T head, Tail... tail) {
    return std::array<T, 1 + sizeof...(Tail)>({head, tail...});
}

// To allow use of -ffast-math we need to have a special check for nan
// that bypasses the "ignore nan" flag
__attribute__((optimize("-fno-finite-math-only"))) static inline bool is_nan(float x) {
    return __builtin_isnan(x);
}

// Round to integer
// Default rounding mode: round to nearest
inline int round_int(float x) {
#ifdef __arm__
    int res;
    asm("vcvtr.s32.f32   %[res], %[x]"
        : [ res ] "=X"(res)
        : [ x ] "w"(x));
    return res;
#else
    return (int)nearbyint(x);
#endif
}

// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
inline float wrap_pm(float x, float y) {
#ifdef FPU_FPV4
    float intval = (float)round_int(x / y);
#else
    float intval = nearbyint(x / y);
#endif
    return x - intval * y;
}

// Same as fmodf but result is positive and y must be positive
inline float fmodf_pos(float x, float y) {
    float res = wrap_pm(x, y);
    if (res < 0) res += y;
    return res;
}

inline float wrap_pm_pi(float x) {
    return wrap_pm(x, 2 * M_PI);
}

// Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta
// as per the magnitude invariant clarke transform
// The magnitude of the alpha-beta vector may not be larger than sqrt(3)/2
// Returns true on success, and false if the input was out of range
inline bool SVM(float alpha, float beta, float* tA, float* tB, float* tC) {
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;
        } break;
    }

    int result_valid =
            *tA >= 0.0f && *tA <= 1.0f
         && *tB >= 0.0f && *tB <= 1.0f
         && *tC >= 0.0f && *tC <= 1.0f;
    return result_valid;
}


// based on https://math.stackexchange.com/a/1105038/81278
inline float fast_atan2(const float y, const float x) {
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = std::abs(y);
    float abs_x = std::abs(x);
    // inject FLT_MIN in denominator to avoid division by zero
    float a = std::min(abs_x, abs_y) / (std::max(abs_x, abs_y) + std::numeric_limits<float>::min());
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if (abs_y > abs_x)
        r = 1.57079637f - r;
    // if x < 0 then r := 3.14159274 - r
    if (x < 0.0f)
        r = 3.14159274f - r;
    // if y < 0 then r := -r
    if (y < 0.0f)
        r = -r;

    return r;
}

// Evaluate polynomials using Fused Multiply Add intrisic instruction.
// coeffs[0] is highest order, as per numpy.polyfit
// p(x) = coeffs[0] * x^deg + ... + coeffs[deg], for some degree "deg"
inline float horner_fma(float x, const float *coeffs, size_t count) {
    float result = 0.0f;
    for (size_t idx = 0; idx < count; ++idx)
        result = (result * x) + coeffs[idx];
    return result;
}

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
inline int mod(const int dividend, const int divisor){
    int r = dividend % divisor;
    return (r < 0) ? (r + divisor) : r;
}

uint32_t deadline_to_timeout(uint32_t deadline_ms);
uint32_t timeout_to_deadline(uint32_t timeout_ms);
int is_in_the_future(uint32_t time_ms);

uint32_t micros(void);
void delay_us(uint32_t us);

extern "C" {
float our_arm_sin_f32(float x);
float our_arm_cos_f32(float x);
}