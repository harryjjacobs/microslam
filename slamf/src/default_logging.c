#include <log/log.h>

#include "slam/logging.h"

void logging_log(int level, const char *file, int line, const char *fmt, ...) {
  char buf[256];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  log_log(level, file, line, "%s", buf);
}