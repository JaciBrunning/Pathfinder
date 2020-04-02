#ifndef PATHFINDER_MATH_UTIL_H_DEF
#define PATHFINDER_MATH_UTIL_H_DEF

#include <math.h>

#define PI 3.14159265358979323846
#define TAU PI*2

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

double bound_radians(double angle);

double r2d(double angleInRads);

double d2r(double angleInDegrees);

#endif