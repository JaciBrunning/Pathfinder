#include "trajectory.h"

void pf_trajectory_copy(Segment *src, Segment *dest, int length) {
    int i;
    for (i = 0; i < length; i++) {
        Segment s = src[i];
        Segment d;
        
        d.dt = s.dt;
        d.x = s.x;
        d.y = s.y;
        d.position = s.position;
        d.velocity = s.velocity;
        d.acceleration = s.acceleration;
        d.jerk = s.jerk;
        d.heading = s.heading;
        
        dest[i] = d;
    }
}

TrajectoryInfo pf_trajectory_prepare(TrajectoryConfig c) {
    double max_a2 = c.max_a * c.max_a;
    double max_j2 = c.max_j * c.max_j;
    
    double checked_max_v = MIN(c.max_v,
        (-(max_a2) + sqrt(max_a2 * max_a2 + 4 * (max_j2 * c.max_a * c.dest_pos)))
        / (2 * c.max_j)
    );
    
    int filter1 = (int) ceil((checked_max_v / c.max_a) / c.dt);
    int filter2 = (int) ceil((c.max_a / c.max_j) / c.dt);
    
    double impulse = (c.dest_pos / checked_max_v) / c.dt;
    int time = (int) ceil(filter1 + filter2 + impulse);
    
    TrajectoryInfo info = { filter1, filter2, time, c.dt, 0, checked_max_v, impulse };
    return info;
}

int pf_trajectory_create(TrajectoryInfo info, TrajectoryConfig c, Segment *seg) {
    int ret = pf_trajectory_fromSecondOrderFilter(info.filter1, info.filter2, info.dt, info.u, info.v, info.impulse, info.length, seg);
    
    if (ret < 0) {
        return ret;
    }
    
    double d_theta = c.dest_theta - c.src_theta;
    int i;
    for (i = 0; i < info.length; i++) {
        seg[i].heading = c.src_theta + d_theta * (seg[i].position) / (seg[info.length - 1].position);
    }
    return 0;
}

int pf_trajectory_fromSecondOrderFilter(int filter_1_l, int filter_2_l, 
        double dt, double u, double v, double impulse, int len, Segment *t) {
    Segment last_section = {dt, 0, 0, 0, u, 0, 0};
    
    if (len < 0) {
        // Error
        return -1;
    }
    
    // double f1_buffer[len];
    double *f1_buffer = (double *) malloc(len * sizeof(double));       // VS doesn't support VLAs
    f1_buffer[0] = (u / v) * filter_1_l;
    double f2;
    
    int i;
    for (i = 0; i < len; i++) {
        double input = MIN(impulse, 1);
        if (input < 1) {
            input -= 1;
            impulse = 0;
        } else {
            impulse -= input;
        }

        double f1_last;
        
        if (i > 0) f1_last = f1_buffer[i - 1];
        else f1_last = f1_buffer[0];

        f1_buffer[i] = MAX(0.0, MIN(filter_1_l, f1_last + input));

        f2 = 0;
        int j;
        for (j = 0; j < filter_2_l; ++j) {
            if (i - j < 0) break;

            f2 += f1_buffer[i - j];
        }
        f2 = f2 / filter_1_l;

        t[i].velocity = f2 / filter_2_l * v;

        t[i].position = (last_section.velocity + t[i].velocity) / 2.0 * dt + last_section.position;

        t[i].x = t[i].position;
        t[i].y = 0;

        t[i].acceleration = (t[i].velocity - last_section.velocity) / dt;
        t[i].jerk = (t[i].acceleration - last_section.acceleration) / dt;
        t[i].dt = dt;

        last_section = t[i];
    }
    free(f1_buffer);
    return 0;
}