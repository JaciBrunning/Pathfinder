#ifndef PATHFINDER_SPLINE_H_DEF
#define PATHFINDER_SPLINE_H_DEF

#include "structs.h"
#include "mathutil.h"

#define PATHFINDER_SAMPLES_FAST (int)1000
#define PATHFINDER_SAMPLES_LOW  (int)PATHFINDER_SAMPLES_FAST*10
#define PATHFINDER_SAMPLES_HIGH (int)PATHFINDER_SAMPLES_LOW*10

Coord pf_spline_coords(Spline s, double percentage);
double pf_spline_deriv(Spline s, double percentage);
double pf_spline_deriv_2(double a, double b, double c, double d, double e, double k, double p);
double pf_spline_angle(Spline s, double percentage);

double pf_spline_distance(Spline *s, int sample_count);
double pf_spline_progress_for_distance(Spline s, double distance, int sample_count);

#endif