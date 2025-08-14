#define _POSIX_C_SOURCE 199309L // for clock_gettime
#include <math.h>
#include <slam/course_to_fine_scan_matching.h>
#include <slam/logging.h>
#include <slam/map.h>
#include <slam/scan.h>
#include <slam/utils.h>
#include <slam/viewer.h>
#include <slam/weighted_scan_matching.h>
#include <time.h>

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

    gt_scan->range[i] = fmin(dist_x, dist_y) + 150.0f * sinf(9.0f * r);
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
    if (angle < 0)
      angle += TWO_PI;
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

bool check_relocalise(robot_pose_t *current_state,
                      robot_pose_t *estimated_state, float relocalise_dist_t,
                      float relocalise_dist_r) {
  float dist_t = hypotf(current_state->pose.x - estimated_state->pose.x,
                        current_state->pose.y - estimated_state->pose.y);
  float dist_r = fabsf(current_state->pose.r - estimated_state->pose.r);
  return (dist_t > relocalise_dist_t || dist_r > relocalise_dist_r);
}

int main() {
  // the distance from the previous update at which the scan matching will
  // update the pose
  const uint16_t update_distance = 50;
  const float update_angle = 0.1f;

  // the maximum distance from the current pose estimate at which the scan
  // matching pose estimate will be considered valid
  const float relocalise_dist_t = 100.0f;
  const float relocalise_dist_r = 0.2f;

  slam_viewer_t viewer;
  slam_viewer_init(&viewer);

  const uint16_t map_size = 4096;
  const uint16_t map_depth = 8;
  const uint16_t map_leaf_size = map_size >> map_depth;

  occupancy_quadtree_t occ;
  occupancy_quadtree_init(&occ, 0, 0, map_size, map_depth);

  scan_t gt_scan, scan;
  scan_reset(&gt_scan);
  scan_reset(&scan);
  generate_gt_scan(&gt_scan);

  robot_t robot;
  robot.lidar.max_range = 1000;
  robot.lidar.range_error = 5;
  robot.lidar.bearing_error = 0.001;

  robot.state.pose.x = 75;
  robot.state.pose.y = 0;
  robot.state.pose.r = 0;
  robot.state.error.x = 0;
  robot.state.error.y = 0;
  robot.state.error.r = 0;

  robot_pose_t gt_state;
  gt_state.pose.x = 75;
  gt_state.pose.y = 0;
  gt_state.pose.r = 0;

  pose_t prev_update_pose = robot.state.pose;

  double dt;
  struct timespec prev_time, current_time;
  clock_gettime(CLOCK_MONOTONIC, &prev_time);

  while (!glfwWindowShouldClose(viewer.window)) {
    slam_viewer_begin_draw();

    // calculate the time since the last frame
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    dt = (current_time.tv_sec - prev_time.tv_sec) +
         (current_time.tv_nsec - prev_time.tv_nsec) / 1e9;
    prev_time = current_time;

    // process input
    uint16_t linear_speed = 300; // mm/s
    float angular_speed = 1.0f;  // rad/s

    motion_t gt_motion;
    gt_motion.dx = 0;
    gt_motion.dy = 0;
    gt_motion.dr = 0;
    gt_motion.error_x = 0.1;  // mm
    gt_motion.error_y = 0.1;  // mm
    gt_motion.error_r = 0.05; // rad

    motion_t motion;
    motion.dx = 0;
    motion.dy = 0;
    motion.dr = 0;

    slam_viewer_key key = slam_viewer_getkey(&viewer);
    switch (key) {
    case slam_viewer_key_up:
      motion.dx = dt * linear_speed * cos(gt_state.pose.r);
      motion.dy = dt * linear_speed * sin(gt_state.pose.r);

      gt_motion.dx = dt * linear_speed *
                     (1 + random_normalf(0, gt_motion.error_x)) *
                     cos(gt_state.pose.r);
      gt_motion.dy = dt * linear_speed *
                     (1 + random_normalf(0, gt_motion.error_y)) *
                     sin(gt_state.pose.r);
      break;
    case slam_viewer_key_down:
      motion.dx = dt * -linear_speed * cos(gt_state.pose.r);
      motion.dy = dt * -linear_speed * sin(gt_state.pose.r);
      gt_motion.dx = dt * linear_speed *
                     (-1 + random_normalf(0, gt_motion.error_x)) *
                     cos(gt_state.pose.r);
      gt_motion.dy = dt * linear_speed *
                     (-1 + random_normalf(0, gt_motion.error_y)) *
                     sin(gt_state.pose.r);
      break;
    case slam_viewer_key_left:
      motion.dr = dt * angular_speed;
      gt_motion.dr =
          dt * angular_speed * (1 + random_normalf(0, gt_motion.error_r));
      break;
    case slam_viewer_key_right:
      motion.dr = dt * -angular_speed;
      gt_motion.dr =
          dt * -angular_speed * (1 + random_normalf(0, gt_motion.error_r));
      break;
    default:
      break;
    }

    if (key != slam_viewer_key_none) {
      // move
      move(&gt_state, &gt_motion);
      move(&robot.state, &motion);

      if ((hypotf(robot.state.pose.x - prev_update_pose.x,
                  robot.state.pose.y - prev_update_pose.y) >=
           update_distance) ||
          (fabsf(robot.state.pose.r - prev_update_pose.r)) >= update_angle) {
        prev_update_pose = robot.state.pose;
        robot.state.error.x += gt_motion.error_x * update_distance;
        robot.state.error.y += gt_motion.error_y * update_distance;
        robot.state.error.r += gt_motion.error_r * update_angle;

        scan_reset(&scan);
        // the scan is generated from the ground truth scan
        // and moved to the ground truth robot pose
        generate_noisy_scan(&robot.lidar, &gt_scan, &scan, &gt_state.pose);

        double entropy = map_entropy(&occ);
        INFO("occupancy map entropy: %f", entropy);

        if (entropy < map_leaf_size * 0.0001f) {
          INFO("map is empty, skipping scan matching");
          // update the occupancy grid
          map_add_scan(&occ, &scan, &robot.state.pose, 0, 1.0);
        } else if (scan.hits > 5) {
          robot_pose_t pose_estimate;
          if (scan_matching_match(&scan, &robot.lidar, &occ, &robot.state.pose,
                                  &pose_estimate, 100) &&
              !check_relocalise(&robot.state, &pose_estimate, relocalise_dist_t,
                                relocalise_dist_r)) {
            INFO("scan match pose estimate: %d %d %f", pose_estimate.pose.x,
                 pose_estimate.pose.y, pose_estimate.pose.r);

            // update the robot pose
            robot.state = pose_estimate;
            // update the occupancy grid
            map_add_scan(&occ, &scan, &robot.state.pose, 0, 1.0);
          } else {
            INFO("scan match failed, relocalising");
            if (course_to_fine_scan_matching_match(
                    &scan, &occ, UINT16_MAX, &robot.state, &pose_estimate)) {
              INFO("course to fine scan match pose estimate: %d %d %f",
                   pose_estimate.pose.x, pose_estimate.pose.y,
                   pose_estimate.pose.r);
              // update the robot pose
              robot.state = pose_estimate;
            }
          }
        }
        INFO("ground truth pose: %d %d %f", gt_state.pose.x, gt_state.pose.y,
             gt_state.pose.r);
        INFO("robot pose: %d %d %f", robot.state.pose.x, robot.state.pose.y,
             robot.state.pose.r);
        INFO("difference: %d %d %f", robot.state.pose.x - gt_state.pose.x,
             robot.state.pose.y - gt_state.pose.y,
             robot.state.pose.r - gt_state.pose.r);

        slam_viewer_draw_scan(&scan, &gt_state.pose, 0, 0, 1);
      }
    }

    // draw
    slam_viewer_draw_occupancy(&occ);
    slam_viewer_draw_robot(&gt_state.pose, 0, 0, 1, 0.5);
    slam_viewer_draw_robot(&robot.state.pose, 0, 1, 0, 1);
    slam_viewer_draw_error(&robot.state.pose, &robot.state.error, 0, 0, 1, 0.5);
    // INFO("error: %d %d %f", robot.state.error.x, robot.state.error.y,
    //      robot.state.error.r);
    slam_viewer_end_draw(&viewer);

    // sleep
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec =
        (1.0 / 60.0) * 1e9 - (current_time.tv_nsec - prev_time.tv_nsec);
    nanosleep(&ts, NULL);
  }

  return 0;
}