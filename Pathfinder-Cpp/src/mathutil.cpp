#include "mathutil.h"

double bound_radians(double angle) {
    double newAngle = fmod(angle, TAU);
    if (newAngle < 0) newAngle = TAU + newAngle;
    return newAngle;
}

double r2d(double angleInRads) {
    return angleInRads * 180 / PI;
}

double d2r(double angleInDegrees) {
    return angleInDegrees * PI / 180;
}