#ifndef PATHFINDER_GENERATOR_H_DEF
#define PATHFINDER_GENERATOR_H_DEF

#include "trajectory.h"
#include "structs.h"
#include "spline.h"
#include "fit.h"
#include <stdlib.h>

int pathfinder_prepare(const Waypoint *path, int path_length, void (*fit)(Waypoint,Waypoint,Spline*), int sample_count, double dt,
        double max_velocity, double max_acceleration, double max_jerk, TrajectoryCandidate *cand);
        
int pathfinder_prepare_LabVIEW(const Waypoint *path, int path_length, int sample_count, double dt,
        double max_velocity, double max_acceleration, double max_jerk);
        
int pathfinder_generate_LabVIEW(Segment *segments);

int pathfinder_generate(TrajectoryCandidate *c, Segment *segments);

#endif