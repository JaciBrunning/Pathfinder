#include "pathfinder.h"
#include <stdio.h>

// CONVERSIONS //
// We're using the manual stuff instead of fwrite on a double array so these files
//    can be preserved on different endianess systems.

void intToBytes(int n, char *bytes) {
    bytes[0] = (n >> 24) & 0xFF;
    bytes[1] = (n >> 16) & 0xFF;
    bytes[2] = (n >> 8) & 0xFF;
    bytes[3] = n & 0xFF;
}

int bytesToInt(char *bytes) {
    int value = 0;
    int i;
    for (i = 0; i < 4; i++) {
        int shift = (4 - 1 - i) * 8;
        value += (bytes[i] & 0x000000FF) << shift;
    }
    return value;
}

void longToBytes(unsigned long long n, char *bytes) {
    bytes[0] = (n >> 56) & 0xFF;
    bytes[1] = (n >> 48) & 0xFF;
    bytes[2] = (n >> 40) & 0xFF;
    bytes[3] = (n >> 32) & 0xFF;
    bytes[4] = (n >> 24) & 0xFF;
    bytes[5] = (n >> 16) & 0xFF;
    bytes[6] = (n >> 8) & 0xFF;
    bytes[7] = n & 0xFF;
}

unsigned long long bytesToLong(char *bytes) {
    unsigned long long value = 0;
    int i;
    for (i = 0; i < 8; i++) {
        int shift = (8 - 1 - i) * 8;
        value += (unsigned long long)(((unsigned long long)(bytes[i] & 0xFF)) << shift);
    }
    return value;
}

double longToDouble(unsigned long long l) {
    double result;
    memcpy(&result, &l, 8);
    return result;
}

unsigned long long doubleToLong(double d) {
    unsigned long long result;
    memcpy(&result, &d, 8);
    return result;
}

void doubleToBytes(double n, char *bytes) {
    longToBytes(doubleToLong(n), bytes);
}

double bytesToDouble(char *bytes) {
    return longToDouble(bytesToLong(bytes));
}

// FUNCTIONS //

int pathfinder_serialize(FILE *fp, Segment *trajectory, int trajectory_length) {
    if (fp == NULL) {
        pathfinder_set_error("File does not exist!");
        return -1;
    }

    if (trajectory == NULL) {
        pathfinder_set_error("Can't serialize to a null buffer");
        return -1;
    }

    char buf_1[4];
    intToBytes(trajectory_length, buf_1);
    fwrite(buf_1, 1, 4, fp);
        
    char buf[8];
    int i;
    for (i = 0; i < trajectory_length; i++) {
        Segment s = trajectory[i];
        
        doubleToBytes(s.dt, buf);
        fwrite(buf, 1, 8, fp);
                
        doubleToBytes(s.x, buf);
        fwrite(buf, 1, 8, fp);
        
        doubleToBytes(s.y, buf);
        fwrite(buf, 1, 8, fp);
        
        doubleToBytes(s.position, buf);
        fwrite(buf, 1, 8, fp);
        
        doubleToBytes(s.velocity, buf);
        fwrite(buf, 1, 8, fp);
        
        doubleToBytes(s.acceleration, buf);
        fwrite(buf, 1, 8, fp);
        
        doubleToBytes(s.jerk, buf);
        fwrite(buf, 1, 8, fp);
        
        doubleToBytes(s.heading, buf);
        fwrite(buf, 1, 8, fp);
    }
    return 0;
}

int pathfinder_deserialize(FILE *fp, Segment *target) {
    if (fp == NULL) {
        pathfinder_set_error("File does not exist!");
        return -1;
    }

    if (target == NULL) {
        pathfinder_set_error("Can't deserialize to a null buffer");
        return -1;
    }

    char buf_1[4];
    fread(buf_1, 1, 4, fp);
    int length = bytesToInt(buf_1);
    
    char buf[8];
    
    int i;
    for (i = 0; i < length; i++) {
        fread(buf, 1, 8, fp);
        double dt = bytesToDouble(buf);
                
        fread(buf, 1, 8, fp);
        double x = bytesToDouble(buf);
        
        fread(buf, 1, 8, fp);
        double y = bytesToDouble(buf);
        
        fread(buf, 1, 8, fp);
        double position = bytesToDouble(buf);
        
        fread(buf, 1, 8, fp);
        double velocity = bytesToDouble(buf);
        
        fread(buf, 1, 8, fp);
        double acceleration = bytesToDouble(buf);
        
        fread(buf, 1, 8, fp);
        double jerk = bytesToDouble(buf);
        
        fread(buf, 1, 8, fp);
        double heading = bytesToDouble(buf);
        
        Segment s = { dt, x, y, position, velocity, acceleration, jerk, heading };
        target[i] = s;
    }
    return length;
}

int pathfinder_serialize_csv(FILE *fp, Segment *trajectory, int trajectory_length) {
    if (fp == NULL) {
        pathfinder_set_error("File does not exist!");
        return -1;
    }

    if (trajectory == NULL) {
        pathfinder_set_error("Can't serialize to a null buffer");
        return -1;
    }

    fputs(CSV_LEADING_STRING, fp);
    
    int i;
    for (i = 0; i < trajectory_length; i++) {
        char buf[1024];
        Segment s = trajectory[i];
        sprintf(buf, "%f,%f,%f,%f,%f,%f,%f,%f\n", s.dt, s.x, s.y, s.position, s.velocity, s.acceleration, s.jerk, s.heading);
        fputs(buf, fp);
    }
    return 0;
}

int pathfinder_deserialize_csv(FILE *fp, Segment *target) {
    if (fp == NULL) {
        pathfinder_set_error("File does not exist!");
        return -1;
    }

    if (target == NULL) {
        pathfinder_set_error("Can't deserialize to a null buffer");
        return -1;
    }
    
    char line[1024];
    int line_n = 0;
    int seg_n = 0;
    while (fgets(line, 1024, fp)) {
        char *tmp = strdup(line);
        if (tmp == NULL) {
            pathfinder_set_error("strdup returned null. Have you run out of RAM, Heap Space, or have been hit by a cosmic ray?");
            return -1;
        }
        if (line_n == 0) { } // Do nothing, first line specifies the headers
        
        char *record;
        record = strtok(tmp, ",");
        double dt   = strtod(record, NULL);
        record = strtok(NULL, ",");
        double x    = strtod(record, NULL);
        record = strtok(NULL, ",");
        double y    = strtod(record, NULL);
        record = strtok(NULL, ",");
        double pos  = strtod(record, NULL);
        record = strtok(NULL, ",");
        double vel  = strtod(record, NULL);
        record = strtok(NULL, ",");
        double acc  = strtod(record, NULL);
        record = strtok(NULL, ",");
        double jerk = strtod(record, NULL);
        record = strtok(NULL, ",");
        double head = strtod(record, NULL);
        
        Segment s = { dt, x, y, pos, vel, acc, jerk, head };
        target[seg_n] = s;
        
        free(tmp);
        
        if (line_n != 0) seg_n++;
        line_n++;
    }
    return seg_n;
}

int pathfinder_get_file_length(FILE *fp) {
    if (fp == NULL) {
        pathfinder_set_error("File does not exist!");
        return -1;
    }

    char line[1024];
    int line_n = 0;
    while (fgets(line, 1024, fp)) {
        line_n++;
        if (strlen(line) >= 1024 - 1) {
            char buf[1024];
            sprintf(buf, "WARNING: Line %d is > 1024 characters long. If read, errors may occur.", line_n);
            pathfinder_set_error(buf);
            return -1;
        }
    }

    rewind(fp);

    return line_n;
}