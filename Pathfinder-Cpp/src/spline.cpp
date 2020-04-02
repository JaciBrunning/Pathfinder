#include "spline.h"

Coord pf_spline_coords(Spline s, double percentage) {
    percentage = MAX(MIN(percentage, 1), 0);
    double x = percentage * s.knot_distance;
    double y = (s.a*x + s.b) * (x*x*x*x) + (s.c*x + s.d) * (x*x) + s.e*x;    // Heh, sex
    
    double cos_theta = cos(s.angle_offset);
    double sin_theta = sin(s.angle_offset);
    
    Coord c = {
        x * cos_theta - y * sin_theta + s.x_offset,
        x * sin_theta + y * cos_theta + s.y_offset
    };
    return c;
}

double pf_spline_deriv(Spline s, double percentage) {
    double x = percentage * s.knot_distance;
    return (5*s.a*x + 4*s.b) * (x*x*x) + (3*s.c*x + 2*s.d) * x + s.e;
}

double pf_spline_deriv_2(double a, double b, double c, double d, double e, double k, double p) {
    double x = p * k;
    return (5*a*x + 4*b) * (x*x*x) + (3*c*x + 2*d) * x + e;
}

double pf_spline_angle(Spline s, double percentage) {
    return bound_radians(atan(pf_spline_deriv(s, percentage)) + s.angle_offset);
}

double pf_spline_distance(Spline *s, int sample_count) {
    double sample_count_d = (double) sample_count;
    
    double a = s->a; double b = s->b; double c = s->c; 
    double d = s->d; double e = s->e; double knot = s->knot_distance;
    
    double arc_length = 0, t = 0, dydt = 0;
    
    double deriv0 = pf_spline_deriv_2(a, b, c, d, e, knot, 0);
    
    double integrand = 0;
    double last_integrand = sqrt(1 + deriv0*deriv0) / sample_count_d;
    
    int i;
    for (i = 0; i <= sample_count; i = i + 1) {
        t = i / sample_count_d;
        dydt = pf_spline_deriv_2(a, b, c, d, e, knot, t);
        integrand = sqrt(1 + dydt*dydt) / sample_count_d;
        arc_length += (integrand + last_integrand) / 2;
        last_integrand = integrand;
    }
    double al = knot * arc_length;
    s->arc_length = al;
    return al;
}

double pf_spline_progress_for_distance(Spline s, double distance, int sample_count) {
    double sample_count_d = (double) sample_count;
    
    double a = s.a; double b = s.b; double c = s.c;
    double d = s.d; double e = s.e; double knot = s.knot_distance;
    
    double arc_length = 0, t = 0, dydt = 0, last_arc_length = 0;
    
    double deriv0 = pf_spline_deriv_2(a, b, c, d, e, knot, 0);

    double integrand = 0;
    double last_integrand = sqrt(1 + deriv0*deriv0) / sample_count_d;
    
    distance /= knot;
    
    int i;
    for (i = 0; i <= sample_count; i = i + 1) {
        t = i / sample_count_d;
        dydt = pf_spline_deriv_2(a, b, c, d, e, knot, t);
        integrand = sqrt(1 + dydt*dydt) / sample_count_d;
        arc_length += (integrand + last_integrand) / 2;
        if (arc_length > distance) break;
        last_integrand = integrand;
        last_arc_length = arc_length;
    }
    
    double interpolated = t;
    if (arc_length != last_arc_length) {
        interpolated += ((distance - last_arc_length)
            / (arc_length - last_arc_length) - 1) / sample_count_d;
    }
    return interpolated;
}