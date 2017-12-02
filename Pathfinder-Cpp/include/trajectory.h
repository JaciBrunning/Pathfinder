#ifndef PATHFINDER_TRAJECTORY_H_DEF
#define PATHFINDER_TRAJECTORY_H_DEF

#include "structs.h"
#include "mathutil.h"
#include <stdlib.h>

void pf_trajectory_copy(Segment *src, Segment *dest, int length);

TrajectoryInfo pf_trajectory_prepare(TrajectoryConfig c);
int pf_trajectory_create(TrajectoryInfo info, TrajectoryConfig c, Segment *seg);
int pf_trajectory_fromSecondOrderFilter(int filter_1_l, int filter_2_l,
        double dt, double u, double v, double impulse, int len, Segment *t);

#endif