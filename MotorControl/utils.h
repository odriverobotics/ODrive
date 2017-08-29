
#pragma once

#ifndef __UTILS_H
#define __UTILS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MACRO_MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MACRO_MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MACRO_CONSTRAIN(val, low, high) (((val) < (low)) ? (low) : ((val > high) ? (high) : (val)))

// Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta
// as per the magnitude invariant clarke transform
// The magnitude of the alpha-beta vector may not be larger than sqrt(3)/2
// Returns 0 on success, and -1 if the input was out of range
int SVM(float alpha, float beta, float *tA, float *tB, float *tC);

//beware of inserting large angles!
float wrap_pm_pi(float theta);
float fast_atan2(float y, float x);

#endif  //__UTILS_H