/*
 * types.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef SLAM_TYPES_H_
#define SLAM_TYPES_H_

#include <stdint.h>

#define LOG_ODDS_OCCUPIED 2
#define LOG_ODDS_FREE -3

typedef struct pose_t {
  int16_t x;
  int16_t y;
  float r;
} pose_t;

typedef struct robot_pose_t {
  pose_t pose;
  pose_t error;
} robot_pose_t;

/**
 * @brief Lidar sensor configuration
 */
typedef struct lidar_sensor_t {
  uint16_t max_range;
  uint16_t range_error;
  float bearing_error;
} lidar_sensor_t;

/**
 * @brief Robot model containing sensor configuration
 */
typedef struct robot_model_t {
  lidar_sensor_t lidar;
} robot_model_t;

#endif
