/*
 * loop_closure.h
 *
 *  Created on: Jul 15, 2025
 *      Author: harryjjacobs
 */

#ifndef SLAM_LOOP_CLOSURE_H_
#define SLAM_LOOP_CLOSURE_H_

#include <slam/occupancy_quadtree.h>
#include <slam/scan.h>
#include <slam/types.h>
#include <stdbool.h>

typedef struct {
  uint16_t min_interval; // minimum distance in key poses
                         // between the current pose and the
                         // candidate key pose for loop
                         // closure

  uint16_t max_t; // maximum distance in mm between key poses to
                  // consider a loop closure

  float max_r; // maximum angle in radians between key poses to
               // consider a loop closure
} loop_closure_params_t;

typedef struct {
  uint16_t last_key_pose_id; // last key pose id used for loop closure (0
  // if no loop closure has been performed yet)
  uint16_t *ids;                // array of key pose ids used for loop closure
  robot_pose_t *trajectory;     // trajectory of the robot for loop closure
  uint16_t trajectory_capacity; // capacity of the trajectory array
  uint16_t trajectory_size;     // current size of the trajectory array
  uint16_t loop_closure_count;  // number of loop closures performed
} loop_closure_t;

void loop_closure_init(loop_closure_t *lc);

void loop_closure_clear(loop_closure_t *lc);

/**
 * @brief Append a new pose to the loop closure trajectory and check for loop
 * closure using the latest scan and occupancy map.
 *
 * @param lc The loop closure structure to update
 * @param pose The current robot pose
 * @param scan The latest scan data
 * @param occ The occupancy quadtree representing the map
 * @param params The SLAM system parameters
 * @param relative_pose The relative pose of the loop closure if detected
 * @return true if a loop closure is detected and relative_pose is updated
 * @return false if no loop closure is detected
 */
bool loop_closure_check(loop_closure_t *lc, const robot_pose_t *pose,
                        const scan_t *scan, occupancy_quadtree_t *occ,
                        const loop_closure_params_t *params,
                        robot_pose_t *relative_pose);

/**
 * @brief Apply the loop closure by correcting the trajectory and corresponding
 * leaf nodes in the occupancy map.
 *
 * @param lc The loop closure structure containing the trajectory
 * @param occ The occupancy quadtree representing the map
 * @param params The SLAM system parameters
 * @param relative_pose The pose correction for the last key pose
 */
bool loop_closure_apply(loop_closure_t *lc, occupancy_quadtree_t *occ,
                        const loop_closure_params_t *params,
                        const robot_pose_t *relative_pose);

#endif // SLAM_LOOP_CLOSURE_H_
