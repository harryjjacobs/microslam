#define _POSIX_C_SOURCE 199309L // for clock_gettime
#include <math.h>
#include <slam/logging.h>
#include <slam/odometry.h>
#include <slam/scan.h>
#include <slam/system.h>
#include <slam/viewer.h>
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

void move(robot_pose_t *state, odometry_t *odometry) {
  state->pose.x += odometry->dx;
  state->pose.y += odometry->dy;
  state->pose.r = rotate(state->pose.r, odometry->dr);
}

static void get_odometry_from_key(slam_viewer_key key, double dt,
                                  robot_pose_t *gt_state,
                                  odometry_t *gt_odometry,
                                  odometry_t *odometry) {
  // process input
  static uint16_t linear_speed = 300; // mm/s
  static float angular_speed = 1.0f;  // rad/s

  switch (key) {
  case slam_viewer_key_up:
    odometry->dx += dt * linear_speed * cos(gt_state->pose.r);
    odometry->dy += dt * linear_speed * sin(gt_state->pose.r);

    gt_odometry->dx += dt * linear_speed *
                       (1 + random_normalf(0, gt_odometry->covariance[0][0])) *
                       cos(gt_state->pose.r);
    gt_odometry->dy = dt * linear_speed *
                      (1 + random_normalf(0, gt_odometry->covariance[1][1])) *
                      sin(gt_state->pose.r);
    break;
  case slam_viewer_key_down:
    odometry->dx += dt * -linear_speed * cos(gt_state->pose.r);
    odometry->dy += dt * -linear_speed * sin(gt_state->pose.r);
    gt_odometry->dx += dt * linear_speed *
                       (-1 + random_normalf(0, gt_odometry->covariance[0][0])) *
                       cos(gt_state->pose.r);
    gt_odometry->dy += dt * linear_speed *
                       (-1 + random_normalf(0, gt_odometry->covariance[1][1])) *
                       sin(gt_state->pose.r);
    break;
  case slam_viewer_key_left:
    odometry->dr += dt * angular_speed;
    gt_odometry->dr += dt * angular_speed *
                       (1 + random_normalf(0, gt_odometry->covariance[2][2]));
    break;
  case slam_viewer_key_right:
    odometry->dr += dt * -angular_speed;
    gt_odometry->dr += dt * -angular_speed *
                       (1 + random_normalf(0, gt_odometry->covariance[2][2]));
    break;
  default:
    break;
  }
}

int main() {
  // the distance from the previous update at which the scan matching will
  // update the pose
  const uint16_t update_distance = 25;
  const float update_angle = 0.1f;

  slam_viewer_t viewer;
  slam_viewer_init(&viewer);

  scan_t gt_scan, scan;
  scan_reset(&gt_scan);
  scan_reset(&scan);
  generate_gt_scan(&gt_scan);

  robot_model_t robot_model;
  robot_model.lidar.max_range = 1000;
  robot_model.lidar.range_error = 2;
  robot_model.lidar.bearing_error = 0.001;

  robot_pose_t gt_state;
  gt_state.pose.x = 0;
  gt_state.pose.y = 0;
  gt_state.pose.r = 0;

  pose_t prev_update_pose = gt_state.pose;

  slam_system_params_t params = {
      .key_pose_distance = 100,
      .key_pose_angle = 0.1f,
      .scan_matching_iterations = 100,
      .relocalise_distance_t = 100,
      .relocalise_distance_r = 0.2f,
      .odometry_error_x = 0.1f,  // mm
      .odometry_error_y = 0.1f,  // mm
      .odometry_error_r = 0.05f, // rad
  };

  slam_system_t system;
  slam_system_init(&system, &params, &robot_model);

  double dt;
  struct timespec prev_time, current_time;
  clock_gettime(CLOCK_MONOTONIC, &prev_time);

  odometry_t gt_odometry;
  slam_odometry_reset_pose(&gt_odometry);
  slam_odometry_covariance_diagonal(&gt_odometry, params.odometry_error_x,
                                    params.odometry_error_y,
                                    params.odometry_error_r);

  odometry_t odometry;
  slam_odometry_reset_pose(&odometry);

  while (!glfwWindowShouldClose(viewer.window)) {
    slam_viewer_begin_draw();

    // calculate the time since the last frame
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    dt = (current_time.tv_sec - prev_time.tv_sec) +
         (current_time.tv_nsec - prev_time.tv_nsec) / 1e9;
    prev_time = current_time;

    slam_odometry_reset_pose(&gt_odometry);

    slam_viewer_key key = slam_viewer_getkey(&viewer);
    get_odometry_from_key(key, dt, &gt_state, &gt_odometry, &odometry);

    if (key != slam_viewer_key_none) {
      // move
      move(&gt_state, &gt_odometry);
      INFO("Ground truth state: (%d, %d, %.9f)", gt_state.pose.x,
           gt_state.pose.y, RAD2DEG(gt_state.pose.r));
      if ((hypotf(gt_state.pose.x - prev_update_pose.x,
                  gt_state.pose.y - prev_update_pose.y) >= update_distance) ||
          (fabsf(gt_state.pose.r - prev_update_pose.r)) >= update_angle) {
        prev_update_pose = gt_state.pose;

        scan_reset(&scan);
        // the scan is generated from the ground truth scan
        // and moved to the ground truth robot pose
        generate_noisy_scan(&robot_model.lidar, &gt_scan, &scan,
                            &gt_state.pose);

        // update the slam system
        slam_system_process(&system, &odometry, &scan, dt);

        slam_odometry_reset_pose(&odometry);
      }
    }

    // draw
    slam_viewer_draw_occupancy(&system.map);
    slam_viewer_draw_robot(&gt_state.pose, 0, 0, 1, 0.5);
    slam_viewer_draw_robot(&system.pose.pose, 1, 0, 0, 0.5);
    slam_viewer_draw_error(&system.pose.pose, &system.pose.error, 0, 0, 1, 0.5);
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