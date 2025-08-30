/*
 * slam.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef SLAM_TYPES_H_
#define SLAM_TYPES_H_

#include <stdlib.h>

#define LOG_ODDS_OCCUPIED 2.0f
#define LOG_ODDS_FREE -3.0f

typedef struct pose_t {
  float x;
  float y;
  float r;
} pose_t;

typedef struct robot_pose_t {
  pose_t pose;
  pose_t error;
} robot_pose_t;

typedef struct lidar_sensor_t {
  float max_range;
  float range_error;
  float bearing_error;
} lidar_sensor_t;

typedef struct scan_t {
  float range[360];
  unsigned short hits; // the number of valid (non-zero) hits in the scan
} scan_t;

typedef struct robot_t {
  lidar_sensor_t lidar;
  robot_pose_t state;
} robot_t;

typedef struct motion_t {
  float dx;
  float dy;
  float dr;
  pose_t error;
} motion_t;

typedef struct occupancy_quadtree_t {
  unsigned char max_depth;
  unsigned char depth;
  float x;
  float y;
  float size;
  float log_odds; // log odds of the occupancy
  enum { OCCUPANCY_FREE, OCCUPANCY_MIXED, OCCUPANCY_OCCUPIED } occupancy;
  struct occupancy_quadtree_t *children[4];
} occupancy_quadtree_t;

typedef struct slam_params_t {
} slam_params_t;

#endif /* SLAM_TYPES_H_ */
