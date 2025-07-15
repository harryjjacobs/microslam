/*
 * slam.h
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

typedef struct lidar_sensor_t {
  uint16_t max_range;
  uint16_t range_error;
  float bearing_error;
} lidar_sensor_t;

typedef struct scan_t {
  uint16_t range[360];
  uint16_t hits;  // the number of valid (non-zero) hits in the scan
} scan_t;

typedef struct robot_t {
  lidar_sensor_t lidar;
  robot_pose_t state;
} robot_t;

typedef struct motion_t {
  uint16_t dx;
  uint16_t dy;
  float dr;
  float error_x;  // mm
  float error_y;  // mm
  float error_r;  // rad
} motion_t;

typedef enum {
  OCCUPANCY_FREE = 0,      // free space
  OCCUPANCY_OCCUPIED = 1,  // occupied space
  OCCUPANCY_MIXED = 2,     // mixed state (some children occupied, some free)
} occupancy_state_t;

typedef struct occupancy_quadtree_t {
  uint16_t id;

  int16_t x;
  int16_t y;
  uint16_t size;
  int16_t log_odds;  // keep as int16_t to save space (assuming range
                     // -1000..1000 fits)
  uint8_t depth;
  uint8_t max_depth;
  uint8_t occupancy;

  // TODO: find a way of referencing the children in a more compact way.
  struct occupancy_quadtree_t *children[4];
} occupancy_quadtree_t;

typedef struct {
  uint16_t key_pose_distance;  // distance in mm to consider a new key pose
  float key_pose_angle;        // angle in radians to consider a new key pose

  // scan matching parameters
  uint16_t scan_matching_iterations;  // maximum number of iterations for scan
                                      // matching

  // relocalisation parameters
  float relocalise_distance_t;  // distance in mm to consider a relocalisation
  float relocalise_distance_r;  // angle in radians to consider a relocalisation
} slam_system_params_t;

typedef struct {
  slam_system_params_t params;  // parameters for the SLAM system
  lidar_sensor_t lidar;         // lidar sensor configuration

  robot_pose_t pose;  // current pose of the robot

  occupancy_quadtree_t map;  // occupancy quadtree map of the environment

  robot_pose_t *key_poses;      // history of key poses for localisation
  uint16_t key_poses_capacity;  // capacity of the key poses array
  uint16_t key_pose_id;         // unique identifier of the current key pose
} slam_system_t;

#endif /* SLAM_TYPES_H_ */
