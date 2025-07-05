
#include "logging.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"

static const char* TAG = "[microslam]";

void logging_log(int level, const char* file, int line, const char* fmt, ...) {
  char buffer[128];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  switch (level) {
    case SLAM_LOG_LEVEL_TRACE:
      ESP_LOGV(TAG, "%s:%d: %s", file, line, buffer);
      break;
    case SLAM_LOG_LEVEL_DEBUG:
      ESP_LOGD(TAG, "%s:%d: %s", file, line, buffer);
      break;
    case SLAM_LOG_LEVEL_INFO:
      ESP_LOGI(TAG, "%s:%d: %s", file, line, buffer);
      break;
    case SLAM_LOG_LEVEL_WARN:
      ESP_LOGW(TAG, "%s:%d: %s", file, line, buffer);
      break;
    case SLAM_LOG_LEVEL_ERROR:
      ESP_LOGE(TAG, "%s:%d: %s", file, line, buffer);
      break;
    case SLAM_LOG_LEVEL_FATAL:
      ESP_LOGE(TAG, "%s:%d: %s", file, line, buffer);
      abort();
      break;
    default:
      ESP_LOGI(TAG, "%s:%d: %s", file, line, buffer);
  }
}

#include "slam/logging.h"
