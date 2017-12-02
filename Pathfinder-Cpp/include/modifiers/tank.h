#ifndef PATHFINDER_MOD_TANK_H_DEF
#define PATHFINDER_MOD_TANK_H_DEF

#include <structs.h>
#include "mathutil.h"

void pathfinder_modify_tank(Segment *original, int length, Segment *left, Segment *right, double wheelbase_width);

#endif