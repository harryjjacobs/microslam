#ifndef MICROSLAM_SLAM_STATE_H
#define MICROSLAM_SLAM_STATE_H

#include "slam/types.h"

typedef struct {
  robot_pose_t pose;  // current pose of the robot

  scan_t scan;  // last scan received from the lidar

  occupancy_quadtree_t map;  // occupancy quadtree map of the environment
} slam_system_t;

#endif
