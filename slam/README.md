# slam library

Simple and lightweight LiDAR-based SLAM library written in C, for running on a microcontroller.

All distances are in millimeters, and all angles are in radians.

## Dependencies

For visualisation and testing, the following dependencies are required:

- libglfw3-dev

This project can be used with [Pixi](https://pixi.sh/latest/) so that dependencies can be managed automatically in a
reproducable and isolated environment. To use Pixi, install it and then run:

```bash
# to install deps and build the project
pixi run build
```

```bash
# to run all tests
pixi run test
```

```bash
# to run the test application
pixi run app
```

## Overview

### Logging

This library uses a logging interface as defined in `logging.h`.

The log level is decided at compile time and can be set by either setting the `SLAM_LOG_LEVEL` preprocessor macro to the desired level, or one of `SLAM_LOG_DEBUG`, `SLAM_LOG_INFO`, `SLAM_LOG_WARN`, `SLAM_LOG_ERROR`, or `SLAM_LOG_FATAL`. `SLAM_LOG_LEVEL` takes precedence. The default log level is `SLAM_LOG_DEBUG`.

The available log levels are:
- `SLAM_LOG_LEVEL_TRACE` (0): Extra verbose debugging information, useful for development.
- `SLAM_LOG_LEVEL_DEBUG` (1): Debugging information, useful for development.
- `SLAM_LOG_LEVEL_INFO` (2): Informational messages, useful for general operation.
- `SLAM_LOG_LEVEL_WARN` (3): Warnings about potential issues that do not stop execution.
- `SLAM_LOG_LEVEL_ERROR` (4): Errors that may affect functionality but do not stop execution.
- `SLAM_LOG_LEVEL_FATAL` (5): Critical errors that stop execution.
- `SLAM_LOG_LEVEL_NONE` (6): No logging output.

The default logging backend is [https://github.com/rxi/log.c](https://github.com/rxi/log.c). A different logging library can be used by removing the `default_logging.c` source file from the build, and replacing it with something that implements the `logging_log_[level]` functions as defined in the `logging.h` header file.

