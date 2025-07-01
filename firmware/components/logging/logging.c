
#include "logging.h"

#include <stdio.h>
#include <stdlib.h>

// Set the log level for the ESP-IDF logging system
#if SLAM_LOG_LEVEL == SLAM_LOG_LEVEL_TRACE
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#elif SLAM_LOG_LEVEL == SLAM_LOG_LEVEL_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#elif SLAM_LOG_LEVEL == SLAM_LOG_LEVEL_INFO
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#elif SLAM_LOG_LEVEL == SLAM_LOG_LEVEL_WARN
#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#elif SLAM_LOG_LEVEL == SLAM_LOG_LEVEL_ERROR
#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
#elif SLAM_LOG_LEVEL == SLAM_LOG_LEVEL_NONE
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#else
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#endif

#include "esp_log.h"

static const char* TAG = "[microslam]";

#include <stdarg.h>
#include <stdio.h>

#include "esp_log.h"

void logging_log(int level, const char* file, int line, const char* fmt, ...) {
  char buffer[256];

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
