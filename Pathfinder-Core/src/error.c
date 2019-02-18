#include "pathfinder/error.h"

#include "string.h"

static char error_buf[256];

char * pathfinder_error() {
  return &error_buf[0];
}

void pathfinder_set_error(const char *msg) {
  strcpy(error_buf, msg);
}