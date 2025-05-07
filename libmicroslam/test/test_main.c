#include <log/log.h>
#include <unity/unity.h>

#include "test_map.h"
#include "test_occupancy_quadtree.h"
#include "test_particle_filter.h"
#include "test_utils.h"

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  log_set_level(LOG_INFO);

  UNITY_BEGIN();

  RUN_TEST(test_calc_bearing_to_point);
  RUN_TEST(test_random_range_uniform);
  RUN_TEST(test_random_range_normalf);
  RUN_TEST(test_random_range_normal);
  RUN_TEST(test_normal_pdf);
  RUN_TEST(test_pose_distance);
  RUN_TEST(test_low_variance_resampling);
  RUN_TEST(test_bearing_likelihood);

  RUN_TEST(test_ray_intersects_aabb_from_inside);
  RUN_TEST(test_ray_intersects_aabb_from_just_outside);
  RUN_TEST(test_ray_intersects_aabb_from_far_outside);
  RUN_TEST(test_ray_intersects_aabb_no_intersection);

  RUN_TEST(test_occupancy_quadtree_init);
  RUN_TEST(test_occupancy_quadtree_update);
  RUN_TEST(test_occupancy_quadtree_update_multiple);
  RUN_TEST(test_occupancy_quadtree_update_depth_2);
  RUN_TEST(test_occupancy_quadtree_update_depth_3);
  RUN_TEST(test_occupancy_quadtree_find);
  RUN_TEST(test_occupancy_quadtree_nearest);
  RUN_TEST(test_occupancy_quadtree_raycast);

  // RUN_TEST(test_map_scan_score);
  RUN_TEST(test_map_scan_match_gradient);
  // RUN_TEST(test_mapping_scan_visualisation);
  // RUN_TEST(test_mapping_occupancy_visualisation);

  UNITY_END();
  return 0;
}
