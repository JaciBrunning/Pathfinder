#include "pathfinder/error.h"

#include "string.h"

#include <stdio.h>

static char error_buf[512];
static int has_error = 0;

char * pathfinder_error() {
  return &error_buf[0];
}

void pathfinder_set_error(const char *msg) {
  has_error++;
  strcpy(error_buf, msg);
  printf("[PATHFINDER ERROR] %s", msg);
}

void pathfinder_clear_errors() {
  has_error = 0;
}

int pathfinder_has_error() {
  return has_error;
}