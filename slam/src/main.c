#include <math.h>
#include <slam/logging.h>
#include <slam/map.h>
#include <slam/scan.h>
#include <slam/scan_matching.h>
#include <slam/utils.h>
#include <slam/viewer.h>

/**
 * @brief Generate a scan inside a roughly square area with some obstacles in
 * the middle
 *
 * @param gt_scan The ground truth scan to generate
 */
void generate_gt_scan(scan_t *gt_scan) {
  const float square_half_size = 1500.0f;
  for (size_t i = 0; i < 360; i++) {
    float r = DEG2RAD(i);
    float dist_x = fabs(square_half_size / cosf(r));
    float dist_y = fabs(square_half_size / sinf(r));

    gt_scan->range[i] = fmin(dist_x, dist_y) + 150.0f * sinf(4.0f * r);
  }

  // add some random obstacles
  for (size_t i = 0; i < 20; i++) {
    float r = random_range_uniformf(0, 2 * PI);
    float dist = random_range_uniformf(500.0f, 1500.0f);
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
void generate_noisy_scan(lidar_sensor_t *lidar, scan_t *gt_scan, scan_t *scan,
                         pose_t *pose) {
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
    if (dist > lidar->max_range) {
      scan_add(scan, scan_idx, 0.0f);
    } else {
      scan_add(scan, scan_idx, dist + random_normalf(0, lidar->range_error));
    }
  }
}

void move(robot_pose_t *state, motion_t *motion) {
  state->pose.x += motion->dx;
  state->pose.y += motion->dy;
  state->pose.r = rotate(state->pose.r, motion->dr);
}

int main() {
  slam_viewer_t viewer;
  slam_viewer_init(&viewer);

  const uint16_t map_size = 4096;
  const uint16_t map_depth = 6;
  const uint16_t map_leaf_size = map_size / (1 << map_depth);

  occupancy_quadtree_t occ;
  occupancy_quadtree_init(&occ, 0, 0, map_size, map_depth);

  const uint16_t scan_matching_min_matches = 3;
  const uint16_t scan_matching_iterations = 1000;

  scan_t gt_scan, scan;
  scan_reset(&gt_scan);
  scan_reset(&scan);
  generate_gt_scan(&gt_scan);

  robot_t robot;
  robot.lidar.max_range = 800;
  robot.lidar.range_error = 0;
  robot.lidar.bearing_error = 0;

  robot.state.pose.x = 75;
  robot.state.pose.y = 0;
  robot.state.pose.r = 0;

  robot_pose_t gt_state;
  gt_state.pose.x = 75;
  gt_state.pose.y = 0;
  gt_state.pose.r = 0;

  // draw ground truth scan
  // slam_viewerbegin_draw(&viewer);
  // slam_viewerdraw_scan(&viewer, &gt_scan, &robot.state.pose, 1, 0, 1);
  // map_add_scan(&occ, &gt_scan, &robot.state.pose, 1.0);
  // slam_viewerdraw_occupancy(&viewer, &occ);
  // slam_viewerend_draw(&viewer);

  while (!glfwWindowShouldClose(viewer.window)) {
    // process input
    uint16_t linear_speed = 10;
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
    motion.error.x = 0;  // mm
    motion.error.y = 0;
    motion.error.r = 0.05;

    slam_viewer_key key = slam_viewer_getkey(&viewer);
    switch (key) {
      case slam_viewer_key_up:
        motion.dx = linear_speed * cos(gt_state.pose.r);
        motion.dy = linear_speed * sin(gt_state.pose.r);

        gt_motion.dx = linear_speed * (1 + random_normalf(0, motion.error.x)) *
                       cos(gt_state.pose.r);
        gt_motion.dy = linear_speed * (1 + random_normalf(0, motion.error.y)) *
                       sin(gt_state.pose.r);
        break;
      case slam_viewer_key_down:
        motion.dx = -linear_speed * cos(gt_state.pose.r);
        motion.dy = -linear_speed * sin(gt_state.pose.r);
        gt_motion.dx = linear_speed * (-1 + random_normalf(0, motion.error.x)) *
                       cos(gt_state.pose.r);
        gt_motion.dy = linear_speed * (-1 + random_normalf(0, motion.error.y)) *
                       sin(gt_state.pose.r);
        break;
      case slam_viewer_key_left:
        motion.dr = angular_speed;
        gt_motion.dr = angular_speed * (1 + random_normalf(0, motion.error.r));
        break;
      case slam_viewer_key_right:
        motion.dr = -angular_speed;
        gt_motion.dr = -angular_speed * (1 + random_normalf(0, motion.error.r));
        break;
      default:
        break;
    }

    if (key != slam_viewer_key_none) {
      // move
      move(&gt_state, &gt_motion);
      move(&robot.state, &motion);

      scan_reset(&scan);
      // the scan is generated from the ground truth scan
      // and moved to the ground truth robot pose
      generate_noisy_scan(&robot.lidar, &gt_scan, &scan, &gt_state.pose);

      float entropy = map_entropy(&occ);
      INFO("occupancy map entropy: %f", entropy);

      if (entropy < map_leaf_size * 0.0005f) {
        INFO("map is empty, skipping scan matching");
        // update the occupancy grid
        map_add_scan(&occ, &scan, &robot.state.pose, 1);
      } else if (scan.hits > 30) {
        // perform scan matching float score = -INFINITY;
        pose_t pose_estimate;
        float score = INFINITY;
        if (scan_matching_match(&occ, &scan, &robot.state.pose, &pose_estimate,
                                &score, scan_matching_min_matches,
                                scan_matching_iterations)) {
          INFO("scan match score: %f", score);
          INFO("scan match pose estimate: %d %d %f", pose_estimate.x,
               pose_estimate.y, pose_estimate.r);
          robot.state.pose.x = pose_estimate.x;
          robot.state.pose.y = pose_estimate.y;
          robot.state.pose.r = pose_estimate.r;

          // update the occupancy grid
          map_add_scan(&occ, &scan, &robot.state.pose, (int32_t)score);
        }
      }

      INFO("ground truth pose: %d %d %f", gt_state.pose.x, gt_state.pose.y,
           gt_state.pose.r);
      INFO("robot pose: %d %d %f", robot.state.pose.x, robot.state.pose.y,
           robot.state.pose.r);
    }

    // draw
    slam_viewer_begin_draw();
    slam_viewer_draw_all(&occ, &robot.state, &gt_state, &scan);
    slam_viewer_end_draw(&viewer);
  }

  return 0;
}