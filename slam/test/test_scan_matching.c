#include <math.h>
#include <slam/course_to_fine_scan_matching.h>
#include <slam/logging.h>
#include <slam/scan.h>
#include <slam/types.h>
#include <slam/utils.h>
#include <slam/viewer.h>
#include <slam/weighted_scan_matching.h>
#include <unity/unity.h>

// /**
//  * @brief Generate a scan inside a roughly square area
//  *
//  * @param gt_scan The ground truth scan to generate
//  */
// void generate_scan(scan_t *gt_scan) {
//   const uint16_t square_half_size = 1500;
//   for (size_t i = 0; i < 360; i++) {
//     float r = DEG2RAD(i);
//     float dist_x = fabs(square_half_size / cosf(r));
//     float dist_y = fabs(square_half_size / sinf(r));

//     gt_scan->range[i] = fmin(dist_x, dist_y) + 150 * sinf(2.0f * r);
//   }
// }
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
void generate_scan(scan_t *gt_scan, float x, float y, float r,
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
 * @brief Generate a scan from the map at a given pose
 */
// void generate_scan(const lidar_sensor_t *lidar, const pose_t *pose,
//                    scan_t *scan) {
//   scan_reset(scan);
//   for (size_t i = 0; i < 360; i++) {
//     float angle = DEG2RAD(i);
//     float dx = (float)(lidar->max_range * cosf(angle));
//     float dy = (float)(lidar->max_range * sinf(angle));

//     // Transform the point to the world coordinates
//     int16_t x = (int16_t)(pose->x + dx * cosf(pose->r) - dy * sinf(pose->r));
//     int16_t y = (int16_t)(pose->y + dx * sinf(pose->r) + dy * cosf(pose->r));

//     INFO("Generating scan for angle %d: (%d, %d) (%d, %d)", i, dx, dy, x, y);

//     // Check if the point is within the lidar range
//   }
// }

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
  lidar.range_error = 5;
  lidar.bearing_error = 0.001f;

  scan_t gt_scan, scan;
  scan_reset(&gt_scan);
  scan_reset(&scan);

  gt_scan.range[0] = leaf_size * 25.0f;
  map_add_scan(&occupancy, &gt_scan, &robot_pose, 0, 1.0);

  pose_t pose;
  TEST_ASSERT(scan_matching_match(&gt_scan, &lidar, &occupancy, &robot_pose,
                                  &pose, 100));
  TEST_ASSERT_INT_WITHIN(1, robot_pose.x, pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, robot_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.r);

  scan.range[0] = gt_scan.range[0] + leaf_size;

  TEST_ASSERT(
      scan_matching_match(&scan, &lidar, &occupancy, &robot_pose, &pose, 100));
  TEST_ASSERT_INT_WITHIN(1, robot_pose.x - leaf_size, pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, robot_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.r);

  scan.range[0] = gt_scan.range[0] - 2.0f * leaf_size;

  TEST_ASSERT(
      scan_matching_match(&scan, &lidar, &occupancy, &robot_pose, &pose, 100));
  TEST_ASSERT_INT_WITHIN(1, robot_pose.x + 2.0f * leaf_size, pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, robot_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.r);
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

  slam_viewer_t viewer;
  slam_viewer_init(&viewer);
  slam_viewer_begin_draw();

  generate_scan(&gt_scan, 0, 0, 0, 1500);
  map_add_scan(&occupancy, &gt_scan, &gt_pose, 0, 1.0);

  robot_pose_t initial_guess;

  initial_guess.pose = gt_pose;
  initial_guess.error.x = 10;
  initial_guess.error.y = 10;
  initial_guess.error.r = DEG2RAD(10);

  // Test matching with a ground truth scan
  pose_t pose;
  TEST_ASSERT(course_to_fine_scan_matching_match(&gt_scan, &lidar, &occupancy,
                                                 &initial_guess, &pose));
  TEST_ASSERT_INT_WITHIN(1, gt_pose.x, pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, gt_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.r);

  // Test matching with a translated scan
  gt_pose.x = leaf_size / 2 + leaf_size * 25;
  gt_pose.y = leaf_size / 2 + leaf_size * 25;
  gt_pose.r = 0.0f;

  generate_scan(&scan, gt_pose.x, gt_pose.y, gt_pose.r, 1500);

  initial_guess.pose.x = 0;
  initial_guess.pose.y = 0;
  initial_guess.pose.r = 0;
  initial_guess.error.x = leaf_size * 50;
  initial_guess.error.y = leaf_size * 50;
  initial_guess.error.r = DEG2RAD(5);
  TEST_ASSERT(course_to_fine_scan_matching_match(&scan, &lidar, &occupancy,
                                                 &initial_guess, &pose));
  TEST_ASSERT_INT_WITHIN(leaf_size / 2, gt_pose.x, pose.x);
  TEST_ASSERT_INT_WITHIN(leaf_size / 2, gt_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, gt_pose.r, pose.r);

  // Test matching with a big translation and rotation
  gt_pose.x = leaf_size / 2 + leaf_size * 200;
  gt_pose.y = leaf_size / 2 + leaf_size * 200;
  gt_pose.r = DEG2RAD(45);

  generate_scan(&scan, gt_pose.x, gt_pose.y, gt_pose.r, 1500);

  initial_guess.pose.x = 0;
  initial_guess.pose.y = 0;
  initial_guess.pose.r = 0;
  initial_guess.error.x = 2000;
  initial_guess.error.y = 2000;
  initial_guess.error.r = DEG2RAD(90);
  TEST_ASSERT(course_to_fine_scan_matching_match(&scan, &lidar, &occupancy,
                                                 &initial_guess, &pose));
  TEST_ASSERT_INT_WITHIN(leaf_size / 2, gt_pose.x, pose.x);
  TEST_ASSERT_INT_WITHIN(leaf_size / 2, gt_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, gt_pose.r, pose.r);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_weighted_scan_matching_simple);
  RUN_TEST(test_course_to_fine_scan_matching);

  UNITY_END();
  return 0;
}
