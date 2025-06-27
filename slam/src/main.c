#include <log/log.h>
#include <math.h>
#include <slam/map.h>
#include <slam/microslam_viewer.h>
#include <slam/scan.h>
#include <slam/scan_matching.h>
#include <slam/utils.h>

/**
 * @brief Generate a scan inside a roughly square area with some obstacles in
 * the middle
 *
 * @param gt_scan The ground truth scan to generate
 */
void generate_gt_scan(scan_t *gt_scan) {
  const float square_half_size = 1.5f;
  for (size_t i = 0; i < 360; i++) {
    float r = DEG2RAD(i);
    float dist_x = fabs(square_half_size / cosf(r));
    float dist_y = fabs(square_half_size / sinf(r));

    gt_scan->range[i] = fmin(dist_x, dist_y) + 0.15f * sinf(4.0f * r);
  }

  // add some random obstacles
  for (size_t i = 0; i < 20; i++) {
    float r = random_range_uniformf(0, 2 * PI);
    float dist = random_range_uniformf(0.5f, 1.5f);
    size_t idx = (size_t)(RAD2DEG(r) + 360) % 360;
    gt_scan->range[idx] = fmin(gt_scan->range[idx], dist);
  }
}

/**
 * @brief Generate a noisy scan from a ground truth scan at a given pose
 *
 * @param gt_scan The ground truth scan
 * @param scan The noisy scan to generate
 * @param pose The pose of the robot
 * @param max_range The maximum range of the sensor
 */
void generate_noisy_scan(scan_t *gt_scan, scan_t *scan, pose_t *pose,
                         float max_range) {
  for (size_t i = 0; i < 360; i++) {
    size_t idx = i;

    // get the x and y of the scan in the world
    float r = DEG2RAD(idx);
    float x = gt_scan->range[idx] * cosf(r);
    float y = gt_scan->range[idx] * sinf(r);

    // get the distance to the scan from the robot
    float dx = x - pose->x;
    float dy = y - pose->y;
    float dist = sqrtf(dx * dx + dy * dy);
    float angle = atan2f(dy, dx) - pose->r;
    if (angle < 0) angle += TWO_PI;
    size_t scan_idx = ((size_t)RAD2DEG(angle)) % 360;

    // add noise to the range
    if (dist > max_range) {
      scan_add(scan, scan_idx, 0.0f);
    } else {
      scan_add(scan, scan_idx, dist + random_normalf(0, scan->range_error));
    }
  }
}

void move(robot_pose_t *state, motion_t *motion) {
  state->pose.x += motion->dx;
  state->pose.y += motion->dy;
  state->pose.r = rotate(state->pose.r, motion->dr);
}

int main() {
  log_set_level(LOG_INFO);

  microslam_viewer_t viewer;
  viewer_init(&viewer);

  const float map_size = 5.0f;
  const float map_depth = 7;
  const float map_leaf_size = map_size / powf(2, map_depth);

  occupancy_quadtree_t occ;
  occupancy_quadtree_init(&occ, 0, 0, map_size, map_depth);

  scan_t gt_scan, scan;
  scan_init(&gt_scan, 0.0f, 0.0f);
  scan_init(&scan, 0.0f, 0.0f);
  // scan_init(&scan, 0.001f, 0.02f);
  generate_gt_scan(&gt_scan);

  robot_t robot;
  robot.lidar.max_range = 0.8f;
  robot.lidar.range_error = 0.0;
  robot.lidar.bearing_error = 0.0;

  robot.state.pose.x = 0.75f;
  robot.state.pose.y = 0;
  robot.state.pose.r = 0;

  robot_pose_t gt_state;
  gt_state.pose.x = 0.75f;
  gt_state.pose.y = 0;
  gt_state.pose.r = 0;

  // draw ground truth scan
  // viewer_begin_draw(&viewer);
  // viewer_draw_scan(&viewer, &gt_scan, &robot.state.pose, 1, 0, 1);
  // map_add_scan(&occ, &gt_scan, &robot.state.pose, 1.0);
  // viewer_draw_occupancy(&viewer, &occ);
  // viewer_end_draw(&viewer);

  while (!glfwWindowShouldClose(viewer.window)) {
    // process input
    float linear_speed = 0.01;
    float angular_speed = 0.02;

    motion_t gt_motion;
    gt_motion.dx = 0;
    gt_motion.dy = 0;
    gt_motion.dr = 0;
    gt_motion.error.x = 0;
    gt_motion.error.y = 0;
    gt_motion.error.r = 0;

    motion_t motion;
    motion.dx = 0;
    motion.dy = 0;
    motion.dr = 0;
    motion.error.x = 0.2;
    motion.error.y = 0.2;
    motion.error.r = 0.05;

    microslam_viewer_key key = viewer_getkey(&viewer);
    switch (key) {
      case microslam_viewer_key_up:
        motion.dx = linear_speed * cos(gt_state.pose.r);
        motion.dy = linear_speed * sin(gt_state.pose.r);

        gt_motion.dx = linear_speed * (1 + random_normalf(0, motion.error.x)) *
                       cos(gt_state.pose.r);
        gt_motion.dy = linear_speed * (1 + random_normalf(0, motion.error.y)) *
                       sin(gt_state.pose.r);
        break;
      case microslam_viewer_key_down:
        motion.dx = -linear_speed * cos(gt_state.pose.r);
        motion.dy = -linear_speed * sin(gt_state.pose.r);
        gt_motion.dx = linear_speed * (-1 + random_normalf(0, motion.error.x)) *
                       cos(gt_state.pose.r);
        gt_motion.dy = linear_speed * (-1 + random_normalf(0, motion.error.y)) *
                       sin(gt_state.pose.r);
        break;
      case microslam_viewer_key_left:
        motion.dr = angular_speed;
        gt_motion.dr = angular_speed * (1 + random_normalf(0, motion.error.r));
        break;
      case microslam_viewer_key_right:
        motion.dr = -angular_speed;
        gt_motion.dr = -angular_speed * (1 + random_normalf(0, motion.error.r));
        break;
      default:
        break;
    }

    if (key != microslam_viewer_key_none) {
      // move
      move(&gt_state, &gt_motion);
      move(&robot.state, &motion);

      scan_reset(&scan);
      // the scan is generated from the ground truth scan
      // and moved to the ground truth robot pose
      generate_noisy_scan(&gt_scan, &scan, &gt_state.pose,
                          robot.lidar.max_range);

      double entropy = map_entropy(&occ);
      log_info("occupancy map entropy: %f", entropy);

      if (entropy < map_leaf_size * 0.1) {
        log_info("map is empty, skipping scan matching");
        // update the occupancy grid
        map_add_scan(&occ, &scan, &robot.state.pose, 1.0);
      } else if (scan.hits > 30) {
        // perform scan matching float score = -INFINITY;
        pose_t pose_estimate;
        float score = INFINITY;
        if (scan_matching_match(&occ, &scan, &robot.state.pose, &pose_estimate,
                                &score, 1000)) {
          log_info("scan match score: %f", score);
          log_info("scan match pose estimate: %f %f %f", pose_estimate.x,
                   pose_estimate.y, pose_estimate.r);
          robot.state.pose.x = pose_estimate.x;
          robot.state.pose.y = pose_estimate.y;
          robot.state.pose.r = pose_estimate.r;

          // update the occupancy grid
          map_add_scan(&occ, &scan, &robot.state.pose, score * 10);
        }
      }

      log_info("ground truth pose: %f %f %f", gt_state.pose.x, gt_state.pose.y,
               gt_state.pose.r);
      log_info("robot pose: %f %f %f", robot.state.pose.x, robot.state.pose.y,
               robot.state.pose.r);
    }

    // draw
    viewer_begin_draw();
    viewer_draw_all(&occ, &robot.state, &gt_state, &scan);
    viewer_end_draw(&viewer);
  }

  return 0;
}