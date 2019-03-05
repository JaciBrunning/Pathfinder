#ifndef PATHFINDER_IO_H_DEF
#define PATHFINDER_IO_H_DEF

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pathfinder/lib.h"

#define CSV_LEADING_STRING "dt,x,y,position,velocity,acceleration,jerk,heading\n"

CAPI void intToBytes(int n, char *bytes);
CAPI int bytesToInt(char *bytes);
CAPI void longToBytes(unsigned long long n, char *bytes);
CAPI unsigned long long bytesToLong(char *bytes);
CAPI double longToDouble(unsigned long long l);
CAPI unsigned long long doubleToLong(double d);
CAPI void doubleToBytes(double n, char *bytes);
CAPI double bytesToDouble(char *bytes);

CAPI int pathfinder_serialize(FILE *fp, Segment *trajectory, int trajectory_length);
CAPI int pathfinder_deserialize(FILE *fp, Segment *target);

CAPI int pathfinder_serialize_csv(FILE *fp, Segment *trajectory, int trajectory_length);
CAPI int pathfinder_deserialize_csv(FILE *fp, Segment *target);

// Note: does not trim header.
CAPI int pathfinder_get_file_length(FILE *fp);

#endif