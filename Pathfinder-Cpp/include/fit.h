#ifndef PATHFINDER_FIT_H_DEF
#define PATHFINDER_FIT_H_DEF

#include "structs.h"
#include "mathutil.h"

void pf_fit_hermite_pre(Waypoint a, Waypoint b, Spline *s);
void pf_fit_hermite_cubic(Waypoint a, Waypoint b, Spline *s);
void pf_fit_hermite_quintic(Waypoint a, Waypoint b, Spline *s);

#define FIT_HERMITE_CUBIC   &pf_fit_hermite_cubic
#define FIT_HERMITE_QUINTIC &pf_fit_hermite_quintic

#endif