/**
 * @file logging.h
 * @author Harry Jacobs and Niko the Robot
 * @brief Logging macros for SLAM system. Compile-time log level control.
 *
 * @copyright Copyright (c) Harry Jacobs 2025
 *
 */

#ifndef SLAM_LOGGING_H
#define SLAM_LOGGING_H

#define SLAM_LOG_LEVEL_TRACE 0
#define SLAM_LOG_LEVEL_DEBUG 1
#define SLAM_LOG_LEVEL_INFO 2
#define SLAM_LOG_LEVEL_WARN 3
#define SLAM_LOG_LEVEL_ERROR 4
#define SLAM_LOG_LEVEL_FATAL 5
#define SLAM_LOG_LEVEL_NONE 6

#if defined(SLAM_LOG_TRACE)
#define SLAM_LOG_LEVEL SLAM_LOG_LEVEL_TRACE
#elif defined(SLAM_LOG_DEBUG)
#define SLAM_LOG_LEVEL SLAM_LOG_LEVEL_DEBUG
#elif defined(SLAM_LOG_INFO)
#define SLAM_LOG_LEVEL SLAM_LOG_LEVEL_INFO
#elif defined(SLAM_LOG_WARN)
#define SLAM_LOG_LEVEL SLAM_LOG_LEVEL_WARN
#elif defined(SLAM_LOG_ERROR)
#define SLAM_LOG_LEVEL SLAM_LOG_LEVEL_ERROR
#elif defined(SLAM_LOG_FATAL)
#define SLAM_LOG_LEVEL SLAM_LOG_LEVEL_FATAL
#elif defined(SLAM_LOG_NONE)
#define SLAM_LOG_LEVEL SLAM_LOG_LEVEL_NONE
#endif

// default log level if not defined
#ifndef SLAM_LOG_LEVEL
#define SLAM_LOG_LEVEL SLAM_LOG_LEVEL_DEBUG
#endif

void logging_log(int level, const char* file, int line, const char* fmt, ...);

#if SLAM_LOG_LEVEL <= SLAM_LOG_LEVEL_TRACE
#define TRACE(...) \
  logging_log(SLAM_LOG_LEVEL_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#else
#define TRACE(...)
#endif

#if SLAM_LOG_LEVEL <= SLAM_LOG_LEVEL_DEBUG
#define DEBUG(...) \
  logging_log(SLAM_LOG_LEVEL_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#else
#define DEBUG(...)
#endif

#if SLAM_LOG_LEVEL <= SLAM_LOG_LEVEL_INFO
#define INFO(...) \
  logging_log(SLAM_LOG_LEVEL_INFO, __FILE__, __LINE__, __VA_ARGS__)
#else
#define INFO(...)
#endif

#if SLAM_LOG_LEVEL <= SLAM_LOG_LEVEL_WARN
#define WARN(...) \
  logging_log(SLAM_LOG_LEVEL_WARN, __FILE__, __LINE__, __VA_ARGS__)
#else
#define WARN(...)
#endif

#if SLAM_LOG_LEVEL <= SLAM_LOG_LEVEL_ERROR
#define ERROR(...) \
  logging_log(SLAM_LOG_LEVEL_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#else
#define ERROR(...)
#endif

#if SLAM_LOG_LEVEL <= SLAM_LOG_LEVEL_FATAL
#define FATAL(...) \
  logging_log(SLAM_LOG_LEVEL_FATAL, __FILE__, __LINE__, __VA_ARGS__)
#else
#define FATAL(...)
#endif

#endif
