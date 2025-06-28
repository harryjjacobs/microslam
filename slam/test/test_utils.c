#include <slam/logging.h>
#include <slam/utils.h>
#include <unity/unity.h>

#define FLOAT_EPSILON 1e-6f

void test_calc_bearing_to_point(void) {
  {
    pose_t a = {0, 0, 0};
    pose_t b = {1, 1, 0};
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, -PI / 4,
                             calc_bearing_to_point(&a, &b));
  }
  {
    pose_t a = {1, 1, 0};
    pose_t b = {0, 0, 0};
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 3 * PI / 4,
                             calc_bearing_to_point(&a, &b));
  }
  {
    // ignores rotation of b
    pose_t a = {1, 1, PI / 4};
    pose_t b = {0, 0, PI / 4};
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, PI_2,
                             calc_bearing_to_point(&a, &b));
  }
}

void test_random_range_uniform() {
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_INT_WITHIN(0.5, 0.5, random_uniformf());
  }
}

void test_random_range_normalf() {
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_FLOAT_WITHIN(0.25, 0.25, random_range_normalf(0, 0.5));
  }
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0, random_range_normalf(-0.5, 0.5));
  }
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_FLOAT_WITHIN(0.005, 0.015, random_range_normalf(0.01, 0.02));
  }
}

void test_random_range_normal() {
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_INT_WITHIN(3, 3, random_range_normal(0, 6));
  }
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_INT_WITHIN(50, 0, random_range_normal(-50, 50));
  }
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_INT_WITHIN(0, 1, random_range_normal(1, 2));
  }
}

void test_normal_pdf() {
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0.39894228, normal_pdf(0, 0, 1));
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0.997355701, normal_pdf(0, 0, 0.4));
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0.0438207512, normal_pdf(0, -1, 0.4));
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 1.85839939e-06,
                           normal_pdf(0, 4, 0.8));
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0,
                           normal_pdf(1.41214, 83.630138, 0.1));
}

void test_pose_distance() {
  {
    pose_t a = {0, 0, 0};
    pose_t b = {0, 0, 0};
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0, pose_distance(&a, &b));
  }
  {
    pose_t a = {0, 0, 0};
    pose_t b = {1, 0, 0};
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 1.0, pose_distance(&a, &b));
  }
  {
    pose_t a = {0, 0, 0};
    pose_t b = {1, 1, 0};
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 1.41421356, pose_distance(&a, &b));
  }
  {
    pose_t a = {2, 3, 0};
    pose_t b = {6, 9, 0};
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 7.21110255, pose_distance(&a, &b));
  }
}

void test_ray_intersects_aabb_from_inside() {
  unsigned char result;
  float t;
  result = ray_intersects_aabb(-1, -1, 1, 1, 0, 0, 1, 1, &t);
  TEST_ASSERT_EQUAL_INT(1, result);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 1, t);
}

void test_ray_intersects_aabb_from_just_outside() {
  unsigned char result;
  float t;
  result = ray_intersects_aabb(-100, -5, 3, 3, 3.01, 3.01, -1, -1, &t);
  TEST_ASSERT_EQUAL_INT(1, result);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0.01, t);
}

void test_ray_intersects_aabb_from_far_outside() {
  unsigned char result;
  float t;
  result = ray_intersects_aabb(-1, -1, 1, 1, -1000, -1000, 1, 1, &t);
  TEST_ASSERT_EQUAL_INT(1, result);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 999, t);
}

void test_ray_intersects_aabb_no_intersection() {
  unsigned char result;
  float t;
  result = ray_intersects_aabb(-1, -1, 1, 1, 2, 2, 3, 3, &t);
  TEST_ASSERT_EQUAL_INT(0, result);
}

void test_cholesky_decomp_3x3() {
  float A[3][3] = {
      {4, 12, -16},
      {12, 37, -43},
      {-16, -43, 98},
  };
  float L[3][3] = {0};

  cholesky_decomp_3x3(A, L);

  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 2.0f, L[0][0]);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 6.0f, L[1][0]);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, -8.0f, L[2][0]);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0.0f, L[0][1]);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 1.0f, L[1][1]);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 5.0f, L[2][1]);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0.0f, L[0][2]);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0.0f, L[1][2]);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 3.0f, L[2][2]);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_calc_bearing_to_point);
  RUN_TEST(test_random_range_uniform);
  RUN_TEST(test_random_range_normalf);
  RUN_TEST(test_random_range_normal);
  RUN_TEST(test_normal_pdf);
  RUN_TEST(test_pose_distance);
  RUN_TEST(test_cholesky_decomp_3x3);

  UNITY_END();
  return 0;
}
