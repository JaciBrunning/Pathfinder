#ifndef PATHFINDER_STRUCT_H_DEF
#define PATHFINDER_STRUCT_H_DEF

typedef struct {
    double x, y, angle;
} Waypoint;

typedef struct {
    double a, b, c, d, e;
    double x_offset, y_offset, angle_offset, knot_distance, arc_length;
} Spline;

typedef struct {
    double x, y;
} Coord;

typedef struct {
    double dt, x, y, position, velocity, acceleration, jerk, heading;
} Segment;

typedef struct {
    double dt, max_v, max_a, max_j, src_v, src_theta, dest_pos, dest_v, dest_theta;
    int sample_count;
} TrajectoryConfig;

typedef struct {
    int filter1, filter2, length;
    double dt, u, v, impulse;
} TrajectoryInfo;

typedef struct {
    Spline *saptr;
    double *laptr;
    double totalLength;
    int length;
    int path_length;
    TrajectoryInfo info;
    TrajectoryConfig config;
} TrajectoryCandidate;

#endif