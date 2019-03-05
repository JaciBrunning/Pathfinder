#include "pathfinder.h"

#include <stdlib.h>

TrajectoryCandidate cand_LV;

int pathfinder_prepare(const Waypoint *path, int path_length, void (*fit)(Waypoint,Waypoint,Spline*), int sample_count, double dt,
        double max_velocity, double max_acceleration, double max_jerk, TrajectoryCandidate *cand) {
    if (path_length < 2) return -1;
    
    cand->saptr = (Spline *)malloc((path_length - 1) * sizeof(Spline));
    cand->laptr = (double *)malloc((path_length - 1) * sizeof(double));

    if (cand->saptr == NULL) {
        pathfinder_set_error("Prepare: could not allocate splines array");
        return -1;
    }

    if (cand->laptr == NULL) {
        pathfinder_set_error("Prepare: could not allocate lengths array");
        return -1;
    }
    double totalLength = 0;
    
    int i;
    for (i = 0; i < path_length-1; i++) {
        Spline s;
        fit(path[i], path[i+1], &s);
        double dist = pf_spline_distance(&s, sample_count);
        cand->saptr[i] = s;
        cand->laptr[i] = dist;
        totalLength += dist;
    }
    
    TrajectoryConfig config = {dt, max_velocity, max_acceleration, max_jerk, 0, path[0].angle,
        totalLength, 0, path[0].angle, sample_count};
    TrajectoryInfo info = pf_trajectory_prepare(config);
    int trajectory_length = info.length;
    
    cand->totalLength = totalLength;
    cand->length = trajectory_length;
    cand->path_length = path_length;
    cand->info = info;
    cand->config = config;
    
    return trajectory_length;
}

/********************************************************************************************
*   LabVIEW memory allocation works different from C and a DLL call requires any memory 
*   be allocated up front for pointers that are used as outputs (i.e. 'trajectory candidate').
*   For the mode we could probably pass out the function pointer to LabVIEW via a function
*   and pass it back in, but for now we can keep it static as cubic. Other option is to
*   pass in a flag selecting which path algorithm, and make it an enum in the LabVIEW API.
*   
*   For the 'trajectory candidate' we don't want to expose it to LabVIEW at all, since we
*   would need to know the sizeof 'Spline' and 'double' on the target and pre-allocate it.
*   Instead, we keep it in the DLL memory and return a length instead of a status so we 
*   an allow LabVIEW to create the segments array.
*********************************************************************************************/
int pathfinder_prepare_LabVIEW(const Waypoint *path, int path_length, int sample_count, double dt,
        double max_velocity, double max_acceleration, double max_jerk)
{
    return pathfinder_prepare(path,path_length,FIT_HERMITE_CUBIC,sample_count,dt,max_velocity,max_acceleration,max_jerk,&cand_LV);
}

int pathfinder_generate_LabVIEW(Segment *segments)
{
    return pathfinder_generate(&cand_LV,segments);
}

int pathfinder_generate(TrajectoryCandidate *c, Segment *segments) {
    int trajectory_length = c->length;
    int path_length = c->path_length;
    double totalLength = c->totalLength;
    
    Spline *splines = (c->saptr);
    double *splineLengths = (c->laptr);
    
    int trajectory_status = pf_trajectory_create(c->info, c->config, segments);
    if (trajectory_status < 0) return trajectory_status;
    
    int spline_i = 0;
    double spline_pos_initial = 0, splines_complete = 0;
    
    int i;
    for (i = 0; i < trajectory_length; ++i) {
        double pos = segments[i].position;

        int found = 0;
        while (!found) {
            double pos_relative = pos - spline_pos_initial;
            if (pos_relative <= splineLengths[spline_i]) {
                Spline si = splines[spline_i];
                double percentage = pf_spline_progress_for_distance(si, pos_relative, c->config.sample_count);
                Coord coords = pf_spline_coords(si, percentage);
                segments[i].heading = pf_spline_angle(si, percentage);
                segments[i].x = coords.x;
                segments[i].y = coords.y;
                found = 1;
            } else if (spline_i < path_length - 2) {
                splines_complete += splineLengths[spline_i];
                spline_pos_initial = splines_complete;
                spline_i += 1;
            } else {
                Spline si = splines[path_length - 2];
                segments[i].heading = pf_spline_angle(si, 1.0);
                Coord coords = pf_spline_coords(si, 1.0);
                segments[i].x = coords.x;
                segments[i].y = coords.y;
                found = 1;
            }
        }
    }
    
    free(c->saptr);
    free(c->laptr);
    
    return trajectory_length;
}