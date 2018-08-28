#include <pathfinder.h>

int main() {
    int POINT_LENGTH = 3;

    Waypoint* points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);

    Waypoint p1 = { -4, -1, d2r(45) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
    Waypoint p2 = { -1, 2, 0 };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
    Waypoint p3 = {  2, 4, 0 };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
    points[0] = p1;
    points[1] = p2;
    points[2] = p3;
    
    TrajectoryCandidate candidate;
    pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);
    free(points);

    int length = candidate.length;
    Segment *trajectory = malloc(length * sizeof(Segment));
    
    pathfinder_generate(&candidate, trajectory);
    
    Segment *frontLeft = (Segment *)malloc(length * sizeof(Segment));
    Segment *frontRight = (Segment *)malloc(length * sizeof(Segment));
    Segment *backLeft = (Segment *)malloc(length * sizeof(Segment);
    Segment *backRight = (Segment *)malloc(length * sizeof(Segment));
    
    double wheelbase_width = 0.6;
    double wheelbase_depth = 0.5;
    
    SWERVE_MODE mode = SWERVE_DEFAULT;

    pathfinder_modify_swerve(trajectory, length, frontLeft, frontRight, backLeft, backRight, wheelbase_width, wheelbase_depth, mode);
    
    // Do something with the trajectories...
    
    free(trajectory);
    free(frontLeft);
    free(frontRight);
    free(backLeft);
    free(backRight);
    return 0;
}
