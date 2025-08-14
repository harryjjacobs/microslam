#ifndef SLAM_SYSTEM_H
#define SLAM_SYSTEM_H

#include <stdio.h>

#include "ekf.h"
#include "loop_closure.h"
#include "map.h"
#include "occupancy_quadtree.h"
#include "types.h"

#define MAP_SIZE 4096
#define MAP_DEPTH 8
#define MAP_LEAF_SIZE (MAP_SIZE >> MAP_DEPTH)

typedef struct {
  uint16_t key_pose_distance; // distance in mm to consider a new key pose
  float key_pose_angle;       // angle in radians to consider a new key pose

  // scan matching parameters
  uint16_t scan_matching_iterations; // maximum number of iterations for scan
                                     // matching

  // relocalisation parameters
  uint16_t relocalise_distance_t; // distance in mm to consider a relocalisation
  float relocalise_distance_r; // angle in radians to consider a relocalisation

  // loop closure parameters
  float odometry_error_x; // odometry error per mm for x coordinate
  float odometry_error_y; // odometry error per mm for y coordinate
  float odometry_error_r; // odometry error per radians for rotation

} slam_system_params_t;

typedef struct {
  slam_system_params_t params; // parameters for the SLAM system

  robot_model_t robot_model; // robot model containing sensor configuration

  robot_pose_t pose; // current pose of the robot

  occupancy_quadtree_t map; // occupancy quadtree map of the environment

  robot_pose_t *key_poses;     // history of key poses for localisation
  uint16_t key_poses_capacity; // capacity of the key poses array
  uint16_t key_pose_id;        // unique identifier of the current key pose

  loop_closure_t loop_closure; // loop closure information
} slam_system_t;

void slam_system_init(slam_system_t *state);

void slam_system_process(slam_system_t *system, pose_t *odometry, scan_t *scan);

#endif
