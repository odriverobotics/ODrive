#ifndef __BBPR_H
#define __BBPR_H

#include "odrive_main.h"

#include <math.h>
#include <stdlib.h>
#include <time.h>

//
// Extract individual real or complex roots from list of quadratic factors
//
int roots(float *a,int n,float *wr,float *wi);

//
// Deflate polynomial 'a' by dividing out 'quad'. Return quotient
// polynomial in 'b' and error metric based on remainder in 'err'.
//
void deflate(float *a,int n,float *b,float *quad,float *err);

//
// Find quadratic factor using Bairstow's method (quadratic Newton method).
// A number of ad hoc safeguards are incorporated to prevent stalls due
// to common difficulties, such as zero slope at iteration point, and
// convergence problems.
//
// Bairstow's method is sensitive to the starting estimate. It is possible
// for convergence to fail or for 'wild' values to trigger an overflow.
//
// It is advisable to institute traps for these problems. (To do!)
//
void find_quad(float *a,int n,float *b,float *quad,float *err, int *iter);

//
// Differentiate polynomial 'a' returning result in 'b'.
//
void diff_poly(float *a,int n,float *b);

//
// Attempt to find a reliable estimate of a quadratic factor using modified
// Bairstow's method with provisions for 'digging out' factors associated
// with multiple roots.
//
// This resursive routine operates on the principal that differentiation of
// a polynomial reduces the order of all multiple roots by one, and has no
// other roots in common with it. If a root of the differentiated polynomial
// is a root of the original polynomial, there must be multiple roots at
// that location. The differentiated polynomial, however, has lower order
// and is easier to solve.
//
// When the original polynomial exhibits convergence problems in the
// neighborhood of some potential root, a best guess is obtained and tried
// on the differentiated polynomial. The new best guess is applied
// recursively on continually differentiated polynomials until failure
// occurs. At this point, the previous polynomial is accepted as that with
// the least number of roots at this location, and its estimate is
// accepted as the root.
//
void recurse(float *a,int n,float *b,int m,float *quad, float *err,int *iter);

//
// Top level routine to manage the determination of all roots of the given
// polynomial 'a', returning the quadratic factors (and possibly one linear
// factor) in 'x'.
//
void get_quads(float *a,int n,float *quad,float *x);

float findRoot(float coeffs[], int n);


#endif
