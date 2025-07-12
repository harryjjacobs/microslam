#include <math.h>
#include <slam/logging.h>
#include <slam/scan.h>
#include <slam/types.h>
#include <slam/utils.h>
#include <slam/viewer.h>
#include <slam/weighted_scan_matching.h>
#include <unity/unity.h>

/**
 * @brief Generate a scan inside a roughly square area
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
}

/**
 * @brief Generate a scan from the map at a given pose
 */
void generate_scan(const lidar_sensor_t *lidar, occupancy_quadtree_t *map,
                   const pose_t *pose, scan_t *scan) {
  scan_reset(scan);
  for (size_t i = 0; i < 360; i++) {
    float angle = DEG2RAD(i);
    float dx = lidar->max_range * cosf(angle);
    float dy = lidar->max_range * sinf(angle);

    // Transform the point to the world coordinates
    float x = pose->x + dx * cosf(pose->r) - dy * sinf(pose->r);
    float y = pose->y + dx * sinf(pose->r) + dy * cosf(pose->r);

    // Raycast to find the distance to the nearest obstacle
    float distance;
    occupancy_quadtree_t *node = occupancy_quadtree_raycast(
        map, x, y, dx, dy, lidar->max_range, &distance);

    if (node) {
      scan_add(scan, i, distance);
    } else {
      scan_add(scan, i, lidar->max_range);
    }
  }
}

void test_scan_matching_simple() {
  const uint16_t map_size = 4096;
  const uint8_t depth = 8;
  const uint8_t leaf_size = map_size >> depth;

  INFO("map size: %hu, depth: %d, leaf size: %d", map_size, depth, leaf_size);

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  pose_t robot_pose = {.x = leaf_size / 2, .y = leaf_size / 2, .r = 0.0f};

  lidar_sensor_t lidar;
  lidar.max_range = 3.0f;
  lidar.range_error = 5;
  lidar.bearing_error = 0.001f;

  scan_t gt_scan, scan;
  scan_reset(&gt_scan);
  scan_reset(&scan);

  gt_scan.range[0] = leaf_size * 25.0f;
  map_add_scan(&occupancy, &gt_scan, &robot_pose, 1.0);

  pose_t pose;
  TEST_ASSERT(
      scan_matching_match(&gt_scan, &lidar, &occupancy, &robot_pose, &pose));
  TEST_ASSERT_INT_WITHIN(1, robot_pose.x, pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, robot_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.r);

  scan.range[0] = gt_scan.range[0] + leaf_size;

  TEST_ASSERT(
      scan_matching_match(&scan, &lidar, &occupancy, &robot_pose, &pose));
  TEST_ASSERT_INT_WITHIN(1, robot_pose.x - leaf_size, pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, robot_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.r);

  scan.range[0] = gt_scan.range[0] - 2.0f * leaf_size;

  TEST_ASSERT(
      scan_matching_match(&scan, &lidar, &occupancy, &robot_pose, &pose));
  TEST_ASSERT_INT_WITHIN(1, robot_pose.x + 2.0f * leaf_size, pose.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, robot_pose.y, pose.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pose.r);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_scan_matching_simple);

  UNITY_END();
  return 0;
}
