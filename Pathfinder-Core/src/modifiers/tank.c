#include "pathfinder.h"

void pathfinder_modify_tank(Segment *original, int length, Segment *left_traj, Segment *right_traj, double wheelbase_width) {
    double w = wheelbase_width / 2;
    
    int i;
    for (i = 0; i < length; i++) {
        Segment seg = original[i];
        Segment left = seg;
        Segment right = seg;
        
        double cos_angle = cos(seg.heading);
        double sin_angle = sin(seg.heading);
        
        left.x = seg.x - (w * sin_angle);
        left.y = seg.y + (w * cos_angle);

        right.x = seg.x + (w * sin_angle);
        right.y = seg.y - (w * cos_angle);

        if (i > 0) {
            Segment last_seg = original[i - 1];
            Segment last_left = left_traj[i - 1];
            Segment last_right = right_traj[i - 1];

            double left_distance = sqrt(
                (left.x - last_left.x) * (left.x - last_left.x)
                + (left.y - last_left.y) * (left.y - last_left.y)
            );
            if ((last_seg.x > seg.x && last_left.x < left.x) || (last_seg.y > seg.y && last_left.y < left.y) ||
                (last_seg.x < seg.x && last_left.x > left.x) || (last_seg.y < seg.y && last_left.y > left.y)) {
                    left_distance = -left_distance;
            }

            double right_distance = sqrt(
                (right.x - last_right.x) * (right.x - last_right.x)
                + (right.y - last_right.y) * (right.y - last_right.y)
            );
            if ((last_seg.x > seg.x && last_right.x < right.x) || (last_seg.y > seg.y && last_right.y < right.y) ||
                (last_seg.x < seg.x && last_right.x > right.x) || (last_seg.y < seg.y && last_right.y > right.y)) {
                     right_distance = -right_distance;
             }

            left.position = last_left.position + left_distance;
            left.velocity = left_distance / seg.dt;
            left.acceleration = (left.velocity - last_left.velocity) / seg.dt;
            left.jerk = (left.acceleration - last_left.acceleration) / seg.dt;

            right.position = last_right.position + right_distance;
            right.velocity = right_distance / seg.dt;
            right.acceleration = (right.velocity - last_right.velocity) / seg.dt;
            right.jerk = (right.acceleration - last_right.acceleration) / seg.dt;
        }
        
        left_traj[i] = left;
        right_traj[i] = right;
    }
}