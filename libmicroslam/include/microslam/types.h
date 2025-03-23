/*
 * microslam.h
 *
 *  Created on: Mar 5, 2023
 *      Author: harryjjacobs
 */

#ifndef MICROSLAM_TYPES_H_
#define MICROSLAM_TYPES_H_

#include <stdlib.h>

#define MAP_WIDTH 10
#define MAP_HEIGHT 10

typedef struct pose_t {
  float x;
  float y;
  float r;
} pose_t;

typedef struct landmarks_t {
  pose_t *poses;
  int *signatures;
  size_t size;
} landmarks_t;

typedef struct state_t {
  pose_t pose;
  pose_t error;
} state_t;

typedef struct sensor_t {
  float range;
  float range_error;
  float fov;
  float bearing_error;
} sensor_t;

typedef struct observation_t {
  float range;
  float range_error;
  float bearing;
  float bearing_error;
} observation_t;

typedef struct observations_t {
  observation_t *observations;
  size_t size;
} observations_t;

typedef struct robot_t {
  state_t state;
  sensor_t sensor;
} robot_t;

typedef struct motion_t {
  float dx;
  float dy;
  float dr;
  pose_t error;
} motion_t;

typedef struct map_t {
  int map[MAP_HEIGHT][MAP_WIDTH];
  landmarks_t landmarks;
} map_t;

typedef struct particle_t {
  state_t state;
  float weight;
} particle_t;

typedef struct particle_filter_params_t {
  size_t num_particles;
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
