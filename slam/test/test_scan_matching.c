#include <math.h>
#include <slam/logging.h>
#include <slam/scan.h>
#include <slam/scan_matching.h>
#include <slam/types.h>
#include <slam/utils.h>
#include <slam/viewer.h>
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

// void test_map_scan_score() {
//   occupancy_quadtree_t occupancy;
//   // 5/2^4 = 0.3125
//   occupancy_quadtree_init(&occupancy, 0, 0, 5.0f, 4);

//   pose_t robot_pose = {.x = 0.0f, .y = 0.0f, .r = 0.0f};

//   scan_t scan;
//   scan_reset(&scan);

//   generate_gt_scan(&scan);
//   map_add_scan(&occupancy, &scan, &robot_pose, 1.0);

//   // use the same ground-truth scan to test the score
//   float score;
//   compute_scan_score(&occupancy, &scan, &robot_pose, &score);
//   INFO("score: %f", score);
//   // TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, score);

//   slam_viewer_t viewer;
//   viewer_init(&viewer);
//   while (!glfwWindowShouldClose(viewer.window)) {
//     viewer_begin_draw(&viewer);
//     viewer_draw_scan(&scan, &robot_pose, 1.0f, 0.0f, 0.0f);
//     viewer_draw_occupancy(&occupancy);
//     viewer_end_draw(&viewer);
//   }
// }

void test_scan_matching_gradient_simple() {
  const uint16_t map_size = 4096;
  const uint8_t depth = 11;
  const uint8_t leaf_size = map_size >> depth;

  INFO("map size: %hu, depth: %d, leaf size: %d", map_size, depth, leaf_size);

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  pose_t robot_pose = {.x = leaf_size / 2, .y = leaf_size / 2, .r = 0.0f};

  scan_t scan;
  scan_reset(&scan);
  scan_add(&scan, 0, leaf_size * 5.0f);

  map_add_scan(&occupancy, &scan, &robot_pose, 1.0);

  // slam_viewer_t viewer;
  // viewer_init(&viewer);
  // while (!glfwWindowShouldClose(viewer.window)) {
  //   viewer_begin_draw(&viewer);
  //   viewer_draw_scan(&scan, &robot_pose, 1.0f, 0.0f, 0.0f);
  //   viewer_draw_occupancy(&occupancy);
  //   viewer_end_draw(&viewer);
  // }

  pose_t gradient;
  float score;
  scan_matching_gradient(&occupancy, &scan, &robot_pose, 0, &gradient, &score);
  TEST_ASSERT_EQUAL(0, gradient.x);
  TEST_ASSERT_EQUAL(0, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0f, gradient.r);

  INFO("gradient: %d %d %.9g", gradient.x, gradient.y, gradient.r);

  // the estimated robot pose is moved slightly to the right which means the
  // scan should be closer to the right edge of the map.
  // this means the gradient should be negative in the x direction since the
  // scan should be moved to the left to match the ground truth scan.
  robot_pose.x = leaf_size * 2;
  scan_matching_gradient(&occupancy, &scan, &robot_pose, 0, &gradient, &score);
  TEST_ASSERT_INT_WITHIN(1, -leaf_size * 2, gradient.x);
  TEST_ASSERT_EQUAL(0.0f, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.r);

  robot_pose.x = leaf_size * 3;
  scan_matching_gradient(&occupancy, &scan, &robot_pose, 0, &gradient, &score);
  TEST_ASSERT_INT_WITHIN(1, -leaf_size * 2, gradient.x);
  TEST_ASSERT_EQUAL(0.0f, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.r);
}

void test_scan_matching_gradient() {
  const uint16_t map_size = 4096;
  const uint8_t depth = 11;
  const uint8_t leaf_size = map_size >> depth;

  INFO("map size: %hu, depth: %d, leaf size: %d", map_size, depth, leaf_size);

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  pose_t robot_pose = {.x = 0, .y = 0, .r = 0.0f};

  scan_t gt_scan, scan;
  scan_reset(&gt_scan);
  scan_reset(&scan);
  generate_gt_scan(&gt_scan);

  map_add_scan(&occupancy, &gt_scan, &robot_pose, 1.0);

  pose_t gradient;
  float score;
  scan_matching_gradient(&occupancy, &gt_scan, &robot_pose, 0, &gradient,
                         &score);
  TEST_ASSERT_INT_WITHIN(1, 0, gradient.x);
  TEST_ASSERT_INT_WITHIN(1, 0, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.0f, gradient.r);

  INFO("gradient: %d %d %.9g", gradient.x, gradient.y, gradient.r);

  // the estimated robot pose is moved slightly to the right which means the
  // scan should be closer to the right edge of the map.
  // this means the gradient should be negative in the x direction since the
  // scan should be moved to the left to match the ground truth scan.
  robot_pose.x = 10;
  scan_matching_gradient(&occupancy, &gt_scan, &robot_pose, 0, &gradient,
                         &score);
  TEST_ASSERT_LESS_THAN(0, gradient.x);
  TEST_ASSERT_EQUAL(0, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-1f, 0.0f, gradient.r);

  robot_pose.x = -20;
  scan_matching_gradient(&occupancy, &gt_scan, &robot_pose, 0, &gradient,
                         &score);

  INFO("gradient: %d %d %.9g", gradient.x, gradient.y, gradient.r);

  TEST_ASSERT_GREATER_THAN(0, gradient.x);
  TEST_ASSERT_EQUAL(0, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-1f, 0.0f, gradient.r);
}

void test_scan_matching() {
  const uint16_t map_size = 4096;
  const uint8_t depth = 11;

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  pose_t robot_pose = {.x = 0.0f, .y = 0.0f, .r = 0.0f};

  scan_t gt_scan, scan;
  scan_reset(&gt_scan);
  scan_reset(&scan);
  generate_gt_scan(&gt_scan);

  map_add_scan(&occupancy, &gt_scan, &robot_pose, 1.0);

  pose_t gradient;
  float score;
  scan_matching_gradient(&occupancy, &gt_scan, &robot_pose, 0, &gradient,
                         &score);
  TEST_ASSERT_INT_WITHIN(1, 0, gradient.x);
  TEST_ASSERT_INT_WITHIN(1, 0, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-1, 0.0f, gradient.r);

  INFO("gradient: %d %d %.9g", gradient.x, gradient.y, gradient.r);

  robot_pose.x = 0.1f;
  uint16_t res = scan_matching_match(&occupancy, &gt_scan, &robot_pose,
                                     &gradient, &score, 1, 200);
  TEST_ASSERT_EQUAL(1, res);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_scan_matching_gradient_simple);
  RUN_TEST(test_scan_matching_gradient);
  RUN_TEST(test_scan_matching);

  UNITY_END();
  return 0;
}
