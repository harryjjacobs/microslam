#include <microslam/microslam.h>
#include <microslam/particle_filter.h>
#include <microslam/utils.h>

#include "unity.h"

#define DOUBLE_EPSILON 1e-8

void test_calc_bearing_to_point(void) {
  {
    pose_t a = {0, 0, 0};
    pose_t b = {1, 1, 0};
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, -PI / 4,
                              calc_bearing_to_point(&a, &b));
  }
  {
    pose_t a = {1, 1, 0};
    pose_t b = {0, 0, 0};
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, -PI - PI / 4,
                              calc_bearing_to_point(&a, &b));
  }
}

void test_random_range_uniform() {
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_INT_WITHIN(0.5, 0.5, random_uniform());
  }
}

void test_random_range_normalf() {
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_DOUBLE_WITHIN(0.25, 0.25, random_range_normalf(0, 0.5));
  }
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 0, random_range_normalf(-0.5, 0.5));
  }
  for (int i = 0; i < 1000; i++) {
    TEST_ASSERT_DOUBLE_WITHIN(0.005, 0.015, random_range_normalf(0.01, 0.02));
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
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 0.39894228, normal_pdf(0, 0, 1));
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 0.997355701, normal_pdf(0, 0, 0.4));
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 0.0438207512,
                            normal_pdf(0, -1, 0.4));
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 1.85839939e-06,
                            normal_pdf(0, 4, 0.8));
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 0,
                            normal_pdf(1.41214, 83.630138, 0.1));
}

void test_pose_distance() {
  {
    pose_t a = {0, 0, 0};
    pose_t b = {0, 0, 0};
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 0, pose_distance(&a, &b));
  }
  {
    pose_t a = {0, 0, 0};
    pose_t b = {1, 0, 0};
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 1.0, pose_distance(&a, &b));
  }
  {
    pose_t a = {0, 0, 0};
    pose_t b = {1, 1, 0};
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 1.41421356,
                              pose_distance(&a, &b));
  }
  {
    pose_t a = {2, 3, 0};
    pose_t b = {6, 9, 0};
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 7.21110255,
                              pose_distance(&a, &b));
  }
}

void test_low_variance_resampling() {
  {
    particle_filter_t particle_filter;
    particle_filter.params.num_particles = 10;
    particle_filter.particles = malloc(sizeof(particle_t) * 10);
    for (size_t i = 0; i < particle_filter.params.num_particles; i++) {
      particle_filter.particles[i].weight =
          1.0 / particle_filter.params.num_particles;
    }
    particle_filter_low_variance_resampling(&particle_filter);
    for (size_t i = 0; i < particle_filter.params.num_particles; i++) {
      TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON,
                                1.0 / particle_filter.params.num_particles,
                                particle_filter.particles[i].weight);
    }
    free(particle_filter.particles);
  }
  {
    int count = 0;
    for (size_t i = 0; i < 100; i++) {
      particle_filter_t particle_filter;
      particle_filter.params.num_particles = 6;
      particle_filter.particles = malloc(sizeof(particle_t) * 6);
      particle_filter.particles[0].weight = 0.01;
      particle_filter.particles[0].state.pose.x = 0;
      particle_filter.particles[1].weight = 0.05;
      particle_filter.particles[1].state.pose.x = 0;
      particle_filter.particles[2].weight = 0.5;
      particle_filter.particles[2].state.pose.x = 1;
      particle_filter.particles[3].weight = 0.1;
      particle_filter.particles[3].state.pose.x = 0;
      particle_filter.particles[4].weight = 0.3;
      particle_filter.particles[4].state.pose.x = 0;
      particle_filter.particles[5].weight = 0.04;
      particle_filter.particles[5].state.pose.x = 0;
      particle_filter_low_variance_resampling(&particle_filter);
      for (size_t j = 0; j < particle_filter.params.num_particles; j++) {
        if (particle_filter.particles[j].state.pose.x == 1) {
          count++;
        }
      }
    }
    TEST_ASSERT_EQUAL_INT32(300, count);
  }
}

void test_bearing_likelihood() {
  {
    pose_t pose = {0, 0, 0};
    pose_t landmark = {1, 0, 0};
    pose_t particle = {0, 0, 0};
    double measured_bearing = calc_bearing_to_point(&pose, &landmark);
    double predicted_bearing = calc_bearing_to_point(&particle, &landmark);
    double likelihood = normal_pdf(measured_bearing, predicted_bearing, 0.1);
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 3.9894228, likelihood);
  }
  {
    pose_t pose = {10, 10, 0};
    pose_t landmark = {0, 0, 0};
    pose_t particle = {5, 5, 0};
    double measured_bearing = calc_bearing_to_point(&pose, &landmark);
    double predicted_bearing = calc_bearing_to_point(&particle, &landmark);
    double likelihood = normal_pdf(measured_bearing, predicted_bearing, 0.1);
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_EPSILON, 3.9894228, likelihood);
  }
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
  RUN_TEST(test_low_variance_resampling);
  RUN_TEST(test_bearing_likelihood);
  UNITY_END();
  return 0;
}
