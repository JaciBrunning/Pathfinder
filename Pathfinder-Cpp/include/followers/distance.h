#ifndef PATHFINDER_FOL_DISTANCE_H_DEF
#define PATHFINDER_FOL_DISTANCE_H_DEF

#include <structs.h>

typedef struct {
    double kp, ki, kd, kv, ka;
} FollowerConfig;

typedef struct {
    double last_error, heading, output;
    int segment, finished;
} DistanceFollower;

double pathfinder_follow_distance(FollowerConfig c, DistanceFollower *follower, Segment *trajectory, int trajectory_length, double distance);

double pathfinder_follow_distance2(FollowerConfig c, DistanceFollower *follower, Segment segment, int trajectory_length, double distance);

#endif