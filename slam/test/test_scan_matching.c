#include <math.h>
#include <slam/logging.h>
#include <slam/microslam_viewer.h>
#include <slam/scan.h>
#include <slam/scan_matching.h>
#include <slam/types.h>
#include <slam/utils.h>
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
 * @brief Generate a noisy scan from a ground truth scan
 *
 * @param gt_scan The ground truth scan
 * @param scan The noisy scan
 */
void generate_noisy_scan(scan_t *gt_scan, scan_t *scan) {
  for (size_t i = 0; i < 360; i++) {
    scan->range[i] =
        gt_scan
            ->range[(int)(i + random_normalf(0, scan->bearing_error)) % 360] +
        random_normalf(0, scan->range_error);
  }
}

// void test_map_scan_score() {
//   occupancy_quadtree_t occupancy;
//   // 5/2^4 = 0.3125
//   occupancy_quadtree_init(&occupancy, 0, 0, 5.0f, 4);

//   pose_t robot_pose = {.x = 0.0f, .y = 0.0f, .r = 0.0f};

//   scan_t scan;
//   scan_init(&scan, 0.0f, 0.0f);

//   generate_gt_scan(&scan);
//   map_add_scan(&occupancy, &scan, &robot_pose, 1.0);

//   // use the same ground-truth scan to test the score
//   float score;
//   compute_scan_score(&occupancy, &scan, &robot_pose, &score);
//   INFO("score: %f", score);
//   // TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, score);

//   microslam_viewer_t viewer;
//   viewer_init(&viewer);
//   while (!glfwWindowShouldClose(viewer.window)) {
//     viewer_begin_draw(&viewer);
//     viewer_draw_scan(&scan, &robot_pose, 1.0f, 0.0f, 0.0f);
//     viewer_draw_occupancy(&occupancy);
//     viewer_end_draw(&viewer);
//   }
// }

void test_scan_matching_gradient_simple() {
  const float map_size = 5.0f;
  const unsigned char depth = 6;
  const float leaf_size = map_size / (1 << depth);

  INFO("map size: %.2f, depth: %d, leaf size: %.9g", map_size, depth,
       leaf_size);

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  pose_t robot_pose = {.x = leaf_size / 2, .y = leaf_size / 2, .r = 0.0f};

  scan_t scan;
  scan_init(&scan, 0.0f, 0.0f);
  scan_add(&scan, 0, leaf_size * 5.0f);

  map_add_scan(&occupancy, &scan, &robot_pose, 1.0);

  // microslam_viewer_t viewer;
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
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0f, gradient.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0f, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0f, gradient.r);

  INFO("gradient: %.9g %.9g %.9g", gradient.x, gradient.y, gradient.r);

  // the estimated robot pose is moved slightly to the right which means the
  // scan should be closer to the right edge of the map.
  // this means the gradient should be negative in the x direction since the
  // scan should be moved to the left to match the ground truth scan.
  robot_pose.x = leaf_size * 0.75f;
  scan_matching_gradient(&occupancy, &scan, &robot_pose, 0, &gradient, &score);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, -0.25 * leaf_size, gradient.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.r);

  robot_pose.x = leaf_size * 8.5f;
  scan_matching_gradient(&occupancy, &scan, &robot_pose, 0, &gradient, &score);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, -8 * leaf_size, gradient.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.r);
}

void test_scan_matching_gradient() {
  const float map_size = 5.0f;
  const unsigned char depth = 8;
  const float leaf_size = map_size / (1 << depth);

  INFO("map size: %.2f, depth: %d, leaf size: %.9g", map_size, depth,
       leaf_size);

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  pose_t robot_pose = {.x = 0.0f, .y = 0.0f, .r = 0.0f};

  scan_t gt_scan, scan;
  scan_init(&gt_scan, 0.0f, 0.0f);
  scan_init(&scan, 0.0f, 0.0f);
  generate_gt_scan(&gt_scan);

  map_add_scan(&occupancy, &gt_scan, &robot_pose, 1.0);

  pose_t gradient;
  float score;
  scan_matching_gradient(&occupancy, &gt_scan, &robot_pose, 0, &gradient,
                         &score);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, gradient.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, gradient.r);

  INFO("gradient: %.9g %.9g %.9g", gradient.x, gradient.y, gradient.r);

  // the estimated robot pose is moved slightly to the right which means the
  // scan should be closer to the right edge of the map.
  // this means the gradient should be negative in the x direction since the
  // scan should be moved to the left to match the ground truth scan.
  robot_pose.x = 0.1f;
  scan_matching_gradient(&occupancy, &gt_scan, &robot_pose, 0, &gradient,
                         &score);
  TEST_ASSERT_LESS_THAN_FLOAT(0.0f, gradient.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.0f, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.0f, gradient.r);

  robot_pose.x = -0.3f;
  scan_matching_gradient(&occupancy, &gt_scan, &robot_pose, 0, &gradient,
                         &score);
  TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, gradient.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, gradient.r);
}

void test_scan_matching() {
  const float map_size = 5.0f;
  const unsigned char depth = 9;
  const float leaf_size = map_size / (1 << depth);

  INFO("map size: %.2f, depth: %d, leaf size: %.9g", map_size, depth,
       leaf_size);

  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, map_size, depth);

  pose_t robot_pose = {.x = 0.0f, .y = 0.0f, .r = 0.0f};

  scan_t gt_scan, scan;
  scan_init(&gt_scan, 0.0f, 0.0f);
  scan_init(&scan, 0.0f, 0.0f);
  generate_gt_scan(&gt_scan);

  map_add_scan(&occupancy, &gt_scan, &robot_pose, 1.0);

  pose_t gradient;
  float score;
  scan_matching_gradient(&occupancy, &gt_scan, &robot_pose, 0, &gradient,
                         &score);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, gradient.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, gradient.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, gradient.r);

  INFO("gradient: %.9g %.9g %.9g", gradient.x, gradient.y, gradient.r);

  robot_pose.x = 0.1f;
  unsigned short res = scan_matching_match(&occupancy, &gt_scan, &robot_pose,
                                           &gradient, &score, 200);
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
