#include <log/log.h>

#include "logging.h"

void logging_log_trace(const char *file, int line, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  log_log(LOG_TRACE, file, line, fmt, args);
  va_end(args);
}
