#include <math.h>
#include <slam/logging.h>
#include <slam/map.h>
#include <slam/microslam_viewer.h>
#include <slam/scan.h>
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

// void test_map_scan_match_gradient() {
//   occupancy_quadtree_t occupancy;
//   occupancy_quadtree_init(&occupancy, 0, 0, 5.0f, 12);

//   pose_t robot_pose = {.x = 0.0f, .y = 0.0f, .r = 0.0f};

//   scan_t gt_scan, scan;
//   scan_init(&gt_scan, 0.0f, 0.0f);
//   scan_init(&scan, 0.0f, 0.0f);
//   generate_gt_scan(&gt_scan);

//   map_add_scan(&occupancy, &gt_scan, &robot_pose, 1.0);

//   pose_t gradient;
//   float score;
//   map_scan_match_gradient(&occupancy, &gt_scan, &robot_pose, &gradient,
//   &score); TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, gradient.x);
//   TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, gradient.y);
//   TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, gradient.r);

//   INFO("gradient: %f %f %f", gradient.x, gradient.y, gradient.r);

//   robot_pose.x = 0.1f;
//   map_scan_match_gradient(&occupancy, &gt_scan, &robot_pose, &gradient,
//   &score); TEST_ASSERT_GREATER_THAN(0.0f, score);
//   TEST_ASSERT_GREATER_THAN_FLOAT(0.01f, gradient.x);
//   TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.y);
//   TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.r);

//   robot_pose.x = -0.1f;
//   map_scan_match_gradient(&occupancy, &gt_scan, &robot_pose, &gradient,
//   &score); TEST_ASSERT_GREATER_THAN(0.0f, score);
//   TEST_ASSERT_LESS_THAN_FLOAT(0.01f, gradient.x);
//   TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.y);
//   TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.r);

//   robot_pose.x = 0.1f;
//   robot_pose.y = 0.1f;
//   map_scan_match_gradient(&occupancy, &gt_scan, &robot_pose, &gradient,
//   &score); TEST_ASSERT_GREATER_THAN(0.0f, score);
//   TEST_ASSERT_GREATER_THAN_FLOAT(0.01f, gradient.x);
//   TEST_ASSERT_GREATER_THAN_FLOAT(0.01f, gradient.y);
//   TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.r);

//   robot_pose.x = -0.1f;
//   robot_pose.y = 0.1f;
//   map_scan_match_gradient(&occupancy, &gt_scan, &robot_pose, &gradient,
//   &score); TEST_ASSERT_GREATER_THAN(0.0f, score);
//   TEST_ASSERT_LESS_THAN_FLOAT(0.01f, gradient.x);
//   TEST_ASSERT_GREATER_THAN_FLOAT(0.01f, gradient.y);
//   TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.r);

//   robot_pose.x = 0.0f;
//   robot_pose.y = 0.0f;
//   robot_pose.r = 0.1f;
//   map_scan_match_gradient(&occupancy, &gt_scan, &robot_pose, &gradient,
//   &score); TEST_ASSERT_GREATER_THAN(0.0f, score);
//   TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.x);
//   TEST_ASSERT_FLOAT_WITHIN(1e-2, 0.0f, gradient.y);
//   TEST_ASSERT_GREATER_THAN_FLOAT(0.04f, gradient.r);
// }

void test_mapping_occupancy_visualisation() {
  occupancy_quadtree_t occupancy;
  occupancy_quadtree_init(&occupancy, 0, 0, 5.0f, 6);
  pose_t robot_pose = {.x = 0.0f, .y = 0.0f, .r = 0.0f};

  scan_t gt_scan, scan;
  scan_init(&gt_scan, 0.0f, 0.0f);
  scan_init(&scan, 0.001f, 0.02f);

  generate_gt_scan(&gt_scan);

  generate_noisy_scan(&gt_scan, &scan);
  map_add_scan(&occupancy, &scan, &robot_pose, 1.0);

  generate_noisy_scan(&gt_scan, &scan);
  map_add_scan(&occupancy, &scan, &robot_pose, 1.0);

  generate_noisy_scan(&gt_scan, &scan);
  map_add_scan(&occupancy, &scan, &robot_pose, 1.0);

  microslam_viewer_t viewer;
  viewer_init(&viewer);

  while (!glfwWindowShouldClose(viewer.window)) {
    viewer_begin_draw(&viewer);
    // viewer_draw_scan(&scan, &robot_pose, 1.0f, 0.0f, 0.0f);
    viewer_draw_occupancy(&occupancy);
    viewer_end_draw(&viewer);
  }
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();

  // RUN_TEST(test_map_scan_score);
  // RUN_TEST(test_map_scan_match_gradient);
  // RUN_TEST(test_mapping_scan_visualisation);
  // RUN_TEST(test_mapping_occupancy_visualisation);

  UNITY_END();
  return 0;
}
