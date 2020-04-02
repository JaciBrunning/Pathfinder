#include "followers/distance.h"

double pathfinder_follow_distance(FollowerConfig c, DistanceFollower *follower, Segment *trajectory, int trajectory_length, double distance) {
    int segment = follower->segment;
    if (segment >= trajectory_length) {
        follower->finished = 1;
        follower->output = 0.0;
        Segment last = trajectory[trajectory_length - 1];
        follower->heading = last.heading;
        return 0.0;
    } else {
        return pathfinder_follow_distance2(c, follower, trajectory[segment], trajectory_length, distance);
    }
}

double pathfinder_follow_distance2(FollowerConfig c, DistanceFollower *follower, Segment s, int trajectory_length, double distance) {
    if (follower->segment < trajectory_length) {
        follower->finished = 0;
        double error = s.position - distance;
        double calculated_value = c.kp * error +
                                  c.kd * ((error - follower->last_error) / s.dt) +
                                  (c.kv * s.velocity + c.ka * s.acceleration);
        
        follower->last_error = error;
        follower->heading = s.heading;
        follower->output = calculated_value;
        follower->segment = follower->segment + 1;
        return calculated_value;
    } else {
        follower->finished = 1;
        return 0.0;
    }
}