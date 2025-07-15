#include <math.h>
#include <slam/course_to_fine_scan_matching.h>
#include <slam/logging.h>
#include <slam/scan.h>
#include <slam/types.h>
#include <slam/utils.h>
#include <slam/viewer.h>
#include <slam/weighted_scan_matching.h>
#include <unity/unity.h>

/**
 * @brief Generate a simulated 360-degree scan of an imaginary room centered at
 * (0, 0), from any pose.
 *
 * @param gt_scan The scan to fill.
 * @param x The x-position of the sensor (in mm).
 * @param y The y-position of the sensor (in mm).
 * @param theta The orientation of the sensor (in radians).
 * @param room_half_size Half the size of the square room (from center to wall,
 * in mm).
 */
void generate_room_scan(scan_t *gt_scan, float x, float y, float r,
                        float room_half_size) {
  const float wall_wave_amplitude = 150.0f;
  const float wall_wave_frequency = 2.0f;

  for (size_t i = 0; i < 360; i++) {
    float local_angle = DEG2RAD(i);
    float global_angle = local_angle + r;

    float dx = cosf(global_angle);
    float dy = sinf(global_angle);

    float dist_x = INFINITY;
    float dist_y = INFINITY;

    float wall_wave_offset =
        wall_wave_amplitude * sinf(wall_wave_frequency * global_angle);

    if (fabsf(dx) > 1e-6f) {
      float wall_pos_plus = room_half_size + wall_wave_offset;
      float wall_pos_minus = -room_half_size + wall_wave_offset;

      float t1 = (wall_pos_plus - x) / dx;
      float t2 = (wall_pos_minus - x) / dx;
      float t = fminf(t1 > 0 ? t1 : INFINITY, t2 > 0 ? t2 : INFINITY);

      dist_x = t;
    }

    if (fabsf(dy) > 1e-6f) {
      float wall_pos_plus = room_half_size + wall_wave_offset;
      float wall_pos_minus = -room_half_size + wall_wave_offset;

      float t1 = (wall_pos_plus - y) / dy;
      float t2 = (wall_pos_minus - y) / dy;
      float t = fminf(t1 > 0 ? t1 : INFINITY, t2 > 0 ? t2 : INFINITY);

      dist_y = t;
    }

    float distance = fminf(dist_x, dist_y);

    gt_scan->range[i] = distance;
  }
}

/**
 * @brief Generate a simulated 360-degree scan of a vertical wall.
 *
 * The wall runs along y-axis between y_start and y_end at global x = wall_x,
 * with sinusoidal horizontal wobble.
 *
 * @param gt_scan The scan to fill.
 * @param robot_x Robot's x position (mm).
 * @param robot_y Robot's y position (mm).
 * @param robot_theta Robot's orientation (radians).
 * @param wall_x X position of the wall (mm).
 * @param y_start Start Y of the wall (mm).
 * @param y_end End Y of the wall (mm).
 * @param wave_amplitude Amplitude of wall wobble (mm).
 * @param wave_length Wavelength of the wobble (mm).
 */
void generate_wall_scan(scan_t *gt_scan, float robot_x, float robot_y,
                        float robot_theta, float wall_x, float y_start,
                        float y_end) {
  const float wave_amplitude = 50.0f;
  const float wave_length = 2.0f;

  for (size_t i = 0; i < 360; i++) {
    float local_angle = DEG2RAD(i);
    float global_angle = local_angle + robot_theta;

    float dx = cosf(global_angle);
    float dy = sinf(global_angle);

    float distance = 0;

    // Handle rays roughly pointing towards wall (avoid division by zero)
    if (fabsf(dx) > 1e-6f) {
      // Compute where the ray crosses vertical plane x = wall_x
      float t = (wall_x - robot_x) / dx;

      if (t > 0) {
        float intersect_y = robot_y + t * dy;

        // Check if intersection is within wall's y-span
        if (intersect_y >= y_start && intersect_y <= y_end) {
          // Apply sinusoidal wobble along wall's length (along y)
          float wave_phase = (intersect_y - y_start) / wave_length * 2.0f * PI;
          float wall_surface_x = wall_x + wave_amplitude * sinf(wave_phase);

          // Recompute distance to the actual wobbly wall surface
          float true_t = (wall_surface_x - robot_x) / dx;

          if (true_t > 0) {
            distance = true_t;
          }
        }
      }
    }

    gt_scan->range[i] = distance;
  }
}

void test_weighted_scan_matching_simple() {
  const uint16_t map_size = 4096;
  const uint8_t depth = 8;
  const uint8_t leaf_size = map_size >> depth;

  INFO("map size: %hu, depth: %d, leaf size: %d", map_size, depth, leaf_size);

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  pose_t robot_pose = {.x = leaf_size / 2, .y = leaf_size / 2, .r = 0.0f};

  lidar_sensor_t lidar;
  lidar.max_range = 3000;
  lidar.range_error = 1;
  lidar.bearing_error = 0.001f;

  scan_t gt_scan, scan;
  scan_reset(&gt_scan);
  scan_reset(&scan);

  gt_scan.range[0] = leaf_size * 25.0f;
  map_add_scan(&occupancy, &gt_scan, &robot_pose, 0, 1.0);

  robot_pose_t pose;
  TEST_ASSERT(scan_matching_match(&gt_scan, &lidar, &occupancy, &robot_pose,
                                  &pose, 100));
  TEST_ASSERT_INT_WITHIN(1, robot_pose.x, pose.pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, robot_pose.y, pose.pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.pose.r);

  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.error.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.error.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.error.r);

  scan.range[0] = gt_scan.range[0] + leaf_size;

  TEST_ASSERT(
      scan_matching_match(&scan, &lidar, &occupancy, &robot_pose, &pose, 100));
  TEST_ASSERT_INT_WITHIN(1, robot_pose.x - leaf_size, pose.pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, robot_pose.y, pose.pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.pose.r);

  scan.range[0] = gt_scan.range[0] - 2.0f * leaf_size;

  TEST_ASSERT(
      scan_matching_match(&scan, &lidar, &occupancy, &robot_pose, &pose, 100));
  TEST_ASSERT_INT_WITHIN(1, robot_pose.x + 2.0f * leaf_size, pose.pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, robot_pose.y, pose.pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.pose.r);
}

void test_course_to_fine_scan_matching() {
  const uint16_t map_size = 4096;
  const uint8_t depth = 10;
  const uint8_t leaf_size = map_size >> depth;

  INFO("map size: %hu, depth: %d, leaf size: %d", map_size, depth, leaf_size);

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  pose_t gt_pose = {.x = leaf_size / 2, .y = leaf_size / 2, .r = 0.0f};

  lidar_sensor_t lidar;
  lidar.max_range = 3000;
  lidar.range_error = 5;
  lidar.bearing_error = 0.001f;

  scan_t gt_scan, scan;
  scan_reset(&gt_scan);
  scan_reset(&scan);

  generate_room_scan(&gt_scan, 0, 0, 0, 1500);
  map_add_scan(&occupancy, &gt_scan, &gt_pose, 0, 1.0);

  robot_pose_t initial_guess;

  initial_guess.pose = gt_pose;
  initial_guess.error.x = 10;
  initial_guess.error.y = 10;
  initial_guess.error.r = DEG2RAD(10);

  // Test matching with a ground truth scan
  pose_t pose;
  TEST_ASSERT(course_to_fine_scan_matching_match(&gt_scan, &occupancy, 0,
                                                 &initial_guess, &pose));
  TEST_ASSERT_INT_WITHIN(1, gt_pose.x, pose.x);
  TEST_ASSERT_INT_WITHIN(1, gt_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.r);

  // Test matching with a translated scan
  gt_pose.x = leaf_size / 2 + leaf_size * 25;
  gt_pose.y = leaf_size / 2 + leaf_size * 25;
  gt_pose.r = 0.0f;

  generate_room_scan(&scan, gt_pose.x, gt_pose.y, gt_pose.r, 1500);

  initial_guess.pose.x = 0;
  initial_guess.pose.y = 0;
  initial_guess.pose.r = 0;
  initial_guess.error.x = leaf_size * 50;
  initial_guess.error.y = leaf_size * 50;
  initial_guess.error.r = DEG2RAD(5);
  TEST_ASSERT(course_to_fine_scan_matching_match(&scan, &occupancy, 0,
                                                 &initial_guess, &pose));
  TEST_ASSERT_INT_WITHIN(leaf_size / 2, gt_pose.x, pose.x);
  TEST_ASSERT_INT_WITHIN(leaf_size / 2, gt_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, gt_pose.r, pose.r);

  // Test matching with a big translation and rotation
  gt_pose.x = leaf_size / 2 + leaf_size * 200;
  gt_pose.y = leaf_size / 2 + leaf_size * 200;
  gt_pose.r = DEG2RAD(45);

  generate_room_scan(&scan, gt_pose.x, gt_pose.y, gt_pose.r, 1500);

  initial_guess.pose.x = 0;
  initial_guess.pose.y = 0;
  initial_guess.pose.r = 0;
  initial_guess.error.x = 2000;
  initial_guess.error.y = 2000;
  initial_guess.error.r = DEG2RAD(90);
  TEST_ASSERT(course_to_fine_scan_matching_match(&scan, &occupancy, 0,
                                                 &initial_guess, &pose));
  TEST_ASSERT_INT_WITHIN(leaf_size / 2, gt_pose.x, pose.x);
  TEST_ASSERT_INT_WITHIN(leaf_size / 2, gt_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, gt_pose.r, pose.r);
}

void test_course_to_fine_scan_matching_max_id() {
  const uint16_t map_size = 4096;
  const uint8_t depth = 10;
  const uint8_t leaf_size = map_size >> depth;

  INFO("map size: %hu, depth: %d, leaf size: %d", map_size, depth, leaf_size);

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  lidar_sensor_t lidar;
  lidar.max_range = 3000;
  lidar.range_error = 5;
  lidar.bearing_error = 0.001f;

  scan_t old_scan, new_scan;
  scan_reset(&old_scan);
  scan_reset(&new_scan);

  pose_t old_pose = {.x = leaf_size / 2, .y = leaf_size / 2, .r = 0.0f};
  generate_wall_scan(&old_scan, old_pose.x, old_pose.y, old_pose.r, 500, -1000,
                     1000);

  map_add_scan(&occupancy, &old_scan, &old_pose, 0, 1.0);

  pose_t new_pose_gt = {.x = leaf_size / 2, .y = leaf_size / 2, .r = 0.0f};
  generate_wall_scan(&new_scan, new_pose_gt.x, new_pose_gt.y, new_pose_gt.r,
                     500, -1000, 1000);

  // the new pose is translated from the ground truth to
  // simulate drift of the robot
  pose_t new_pose = {.x = leaf_size / 2 + leaf_size * 50,
                     .y = leaf_size / 2 + leaf_size * 50,
                     .r = 0.0f};

  // we add the new scan at the drifted pose as it would be
  // in the real scenario
  map_add_scan(&occupancy, &new_scan, &new_pose, 1, 1.0);

  robot_pose_t initial_guess;
  initial_guess.pose = new_pose;
  initial_guess.error.x = leaf_size * 200;
  initial_guess.error.y = leaf_size * 200;
  initial_guess.error.r = DEG2RAD(5);

  // try to match the new scan against the map without max_id, with
  // an initial guess that is close to the new pose - it should
  // match with the new scan
  pose_t pose_estimate;
  TEST_ASSERT(course_to_fine_scan_matching_match(
      &new_scan, &occupancy, UINT16_MAX, &initial_guess, &pose_estimate));
  TEST_ASSERT_INT_WITHIN(1, new_pose.x, pose_estimate.x);
  TEST_ASSERT_INT_WITHIN(1, new_pose.y, pose_estimate.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, new_pose.r, pose_estimate.r);

  // try to match the new scan against the map with max_id used to
  // ignore the new scan - it should match with the old scan
  TEST_ASSERT(course_to_fine_scan_matching_match(
      &new_scan, &occupancy, 0, &initial_guess, &pose_estimate));
  TEST_ASSERT_INT_WITHIN(1, old_pose.x, pose_estimate.x);
  TEST_ASSERT_INT_WITHIN(1, old_pose.y, pose_estimate.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, old_pose.r, pose_estimate.r);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_weighted_scan_matching_simple);
  RUN_TEST(test_course_to_fine_scan_matching);
  RUN_TEST(test_course_to_fine_scan_matching_max_id);

  UNITY_END();
  return 0;
}
