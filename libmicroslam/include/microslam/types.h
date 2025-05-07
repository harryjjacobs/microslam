/*
 * microslam.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef MICROSLAM_TYPES_H_
#define MICROSLAM_TYPES_H_

#include <stdlib.h>

#define LOG_ODDS_OCCUPIED 2.0f
#define LOG_ODDS_FREE -3.0f

typedef struct pose_t {
  float x;
  float y;
  float r;
} pose_t;

typedef struct state_t {
  pose_t pose;
  pose_t error;
} state_t;

typedef struct lidar_sensor_t {
  float max_range;
  float range_error;
  float bearing_error;
} lidar_sensor_t;

typedef struct scan_t {
  float range[360];
  float range_error;
  float bearing_error;
} scan_t;

typedef struct robot_t {
  lidar_sensor_t lidar;
  state_t state;
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
  float log_odds;  // log odds of the occupancy
  enum { OCCUPANCY_FREE, OCCUPANCY_MIXED, OCCUPANCY_OCCUPIED } occupancy;
  struct occupancy_quadtree_t *children[4];
} occupancy_quadtree_t;

typedef struct particle_t {
  state_t state;
  float weight;
} particle_t;

typedef struct particle_filter_params_t {
  size_t num_particles;
  float map_width;
  float map_height;
  float initial_rotation;  // initial rotation of the particles
  // range +- to use for the initial rotation of the particles
  float random_rotation_range;
} particle_filter_params_t;

typedef struct particle_filter_t {
  particle_t *particles;
  particle_filter_params_t params;
  state_t state;
} particle_filter_t;

typedef struct microslam_params_t {
  size_t particles;

} microslam_params_t;

#endif /* MICROSLAM_TYPES_H_ */
