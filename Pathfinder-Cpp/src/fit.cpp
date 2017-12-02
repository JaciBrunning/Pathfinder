#include "fit.h"

void pf_fit_hermite_cubic(Waypoint a, Waypoint b, Spline *s) {
    pf_fit_hermite_pre(a, b, s);
    
    double a0_delta = tan(bound_radians(a.angle - (*s).angle_offset));
    double a1_delta = tan(bound_radians(b.angle - (*s).angle_offset));
    
    (*s).a = 0; (*s).b = 0;
    (*s).c = (a0_delta + a1_delta) / ((*s).knot_distance * (*s).knot_distance);
    (*s).d = -(2 * a0_delta + a1_delta) / (*s).knot_distance;
    (*s).e = a0_delta;
}

void pf_fit_hermite_quintic(Waypoint a, Waypoint b, Spline *s) {
    pf_fit_hermite_pre(a, b, s);
    
    double a0_delta = tan(bound_radians(a.angle - (*s).angle_offset));
    double a1_delta = tan(bound_radians(b.angle - (*s).angle_offset));
    
    double d = (*s).knot_distance;
    
    (*s).a = -(3 * (a0_delta + a1_delta)) / (d*d*d*d);
    (*s).b = (8 * a0_delta + 7 * a1_delta) / (d*d*d);
    (*s).c = -(6 * a0_delta + 4 * a1_delta) / (d*d);
    (*s).d = 0; (*s).e = a0_delta;
}

void pf_fit_hermite_pre(Waypoint a, Waypoint b, Spline *s) {
    (*s).x_offset = a.x;
    (*s).y_offset = a.y;
    
    double delta = sqrt( (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y) );
    (*s).knot_distance = delta;
    (*s).angle_offset = atan2(b.y - a.y, b.x - a.x);
}