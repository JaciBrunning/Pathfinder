#include "modifiers/swerve.h"

void pf_modify_swerve_default(Segment *original, int length, Segment *front_left, Segment *front_right,
    Segment *back_left, Segment *back_right, double wheelbase_width, double wheelbase_depth) {
    
    int i;
    for (i = 0; i < length; i++) {
        Segment seg = original[i];
        Segment fl = seg;
        Segment fr = seg;
        Segment bl = seg;
        Segment br = seg;
        
        fl.x = seg.x - wheelbase_width / 2;
        fl.y = seg.y + wheelbase_depth / 2;
        fr.x = seg.x + wheelbase_width / 2;
        fr.y = seg.y + wheelbase_depth / 2;
        
        bl.x = seg.x - wheelbase_width / 2;
        bl.y = seg.y - wheelbase_depth / 2;
        br.x = seg.x + wheelbase_width / 2;
        br.y = seg.y - wheelbase_depth / 2;
        
        front_left[i] = fl;
        front_right[i] = fr;
        back_left[i] = bl;
        back_right[i] = br;
    }
}

void pathfinder_modify_swerve(Segment *original, int length, Segment *front_left, Segment *front_right,
    Segment *back_left, Segment *back_right, double wheelbase_width, double wheelbase_depth, SWERVE_MODE mode) {
    
    if (mode == SWERVE_DEFAULT) {
        pf_modify_swerve_default(original, length, front_left, front_right, back_left, back_right, wheelbase_width, wheelbase_depth);
    }
}
