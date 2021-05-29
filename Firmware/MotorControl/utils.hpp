#pragma once

#include <stdint.h>
#include <limits>
#include <algorithm>
#include <array>
#include <tuple>
#include <cmath>

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

// Function prototypes for implementations in utils.cpp
std::tuple<float, float, float, bool> SVM(float alpha, float beta);
float fast_atan2(float y, float x);
uint32_t deadline_to_timeout(uint32_t deadline_ms);
uint32_t timeout_to_deadline(uint32_t timeout_ms);
int is_in_the_future(uint32_t time_ms);
uint32_t micros(void);
void delay_us(uint32_t us);

extern "C" {
float our_arm_sin_f32(float x);
float our_arm_cos_f32(float x);
}

// ----------------
// Inline functions

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
__attribute__((optimize("-fno-finite-math-only")))
inline bool is_nan(float x) {
    return __builtin_isnan(x);
}

// Round to integer
// Default rounding mode: round to nearest
inline int round_int(float x) {
#ifdef __arm__
    int res;
    asm("vcvtr.s32.f32   %[res], %[x]"
        : [res] "=X" (res)
        : [x] "w" (x) );
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
    float intval = nearbyintf(x / y);
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

// Evaluate polynomials in an efficient way
// coeffs[0] is highest order, as per numpy.polyfit
// p(x) = coeffs[0] * x^deg + ... + coeffs[deg], for some degree "deg"
inline float horner_poly_eval(float x, const float *coeffs, size_t count) {
    float result = 0.0f;
    for (size_t idx = 0; idx < count; ++idx)
        result = (result * x) + coeffs[idx];
    return result;
}

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
inline int mod(const int dividend, const int divisor){
    int r = dividend % divisor;
    if (r < 0) r += divisor;
    return r;
}
