#include "slam/loop_closure.h"

#include <math.h>
#include <slam/course_to_fine_scan_matching.h>
#include <slam/logging.h>
#include <slam/utils.h>

static void grow_trajectory(loop_closure_t *lc) {
  size_t new_capacity =
      lc->trajectory_capacity ? lc->trajectory_capacity * 2 : 16;
  robot_pose_t *new_trajectory = (robot_pose_t *)realloc(
      lc->trajectory, sizeof(robot_pose_t) * new_capacity);
  if (!new_trajectory) {
    FATAL("Failed to allocate memory for loop closure trajectory");
  }
  uint16_t *new_ids =
      (uint16_t *)realloc(lc->ids, sizeof(uint16_t) * new_capacity);
  if (!new_ids) {
    FATAL("Failed to allocate memory for loop closure IDs");
  }
  lc->trajectory = new_trajectory;
  lc->ids = new_ids;
  lc->trajectory_capacity = new_capacity;
}

void loop_closure_init(loop_closure_t *lc) {
  lc->last_key_pose_id = 0;
  lc->trajectory = NULL;
  lc->ids = NULL;
  lc->trajectory_capacity = 0;
  lc->trajectory_size = 0;
  lc->loop_closure_count = 0;
}

void loop_closure_clear(loop_closure_t *lc) {
  if (lc->trajectory) {
    free(lc->trajectory);
    lc->trajectory = NULL;
  }
  if (lc->ids) {
    free(lc->ids);
    lc->ids = NULL;
  }
  lc->trajectory_capacity = 0;
  lc->trajectory_size = 0;
  lc->last_key_pose_id = 0;
  lc->loop_closure_count = 0;
}

bool loop_closure_check(loop_closure_t *lc, const robot_pose_t *pose,
                        const scan_t *scan, occupancy_quadtree_t *occ,
                        const loop_closure_params_t *params,
                        robot_pose_t *relative_pose) {
  if (lc->trajectory_size >= lc->trajectory_capacity) {
    // reallocate memory for the trajectory
    grow_trajectory(lc);
  }

  // add the new key pose
  lc->ids[lc->trajectory_size] = lc->last_key_pose_id;
  lc->trajectory[lc->trajectory_size] = *pose;
  lc->trajectory_size++;

  if (lc->trajectory_size < 2) {
    return false; // not enough poses for loop closure
  }

  if (lc->ids[lc->trajectory_size - 1] - lc->last_key_pose_id <
      params->min_interval) {
    return false; // not enough distance since last loop closure
  }

  // match the latest pose against the map
  // TODO: do we need to update the error of the pose to widen the search area?
  robot_pose_t new_pose;
  if (!course_to_fine_scan_matching_match(
          scan, occ, lc->ids[lc->trajectory_size - 1] - params->min_interval,
          pose, &new_pose)) {
    return false; // scan matching failed
  }

  INFO("Loop closure detected at pose %zu with relative pose (%d, %d, %.6f)!",
       lc->trajectory_size - 1, relative_pose->pose.x, relative_pose->pose.y,
       relative_pose->pose.r);

  // update the last key pose ID
  lc->last_key_pose_id = lc->ids[lc->trajectory_size - 1];
  lc->loop_closure_count++;

  // calculate the relative pose
  relative_pose->pose.x = new_pose.pose.x - pose->pose.x;
  relative_pose->pose.y = new_pose.pose.y - pose->pose.y;
  relative_pose->pose.r = wrap_angle(new_pose.pose.r - pose->pose.r);

  return true;
}

bool loop_closure_apply(loop_closure_t *lc, occupancy_quadtree_t *occ,
                        const loop_closure_params_t *params,
                        const robot_pose_t *relative_pose) {
  if (lc->trajectory_size < 2) {
    return false; // not enough poses to apply loop closure
  }

  // TODO: pose graph optimisation to correct the trajectory
  return true; // placeholder for actual implementation
}
