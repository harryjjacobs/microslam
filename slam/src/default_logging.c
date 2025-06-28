#include <log/log.h>

#include "slam/logging.h"

void logging_log(int level, const char *file, int line, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  log_log(level, file, line, fmt, args);
  va_end(args);
}