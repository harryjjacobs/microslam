#include <microslam/particle_filter.h>
#include <unity/unity.h>

#define FLOAT_EPSILON 1e-6f

void test_low_variance_resampling() {
  {
    particle_filter_t particle_filter;
    particle_filter.params.num_particles = 10;
    particle_filter.particles = (particle_t*)malloc(sizeof(particle_t) * 10);
    for (size_t i = 0; i < particle_filter.params.num_particles; i++) {
      particle_filter.particles[i].weight =
          1.0 / particle_filter.params.num_particles;
    }
    particle_filter_low_variance_resampling(&particle_filter);
    for (size_t i = 0; i < particle_filter.params.num_particles; i++) {
      TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON,
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
      particle_filter.particles = (particle_t*)malloc(sizeof(particle_t) * 6);
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
      free(particle_filter.particles);
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
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 3.9894228, likelihood);
  }
  {
    pose_t pose = {10, 10, 0};
    pose_t landmark = {0, 0, 0};
    pose_t particle = {5, 5, 0};
    double measured_bearing = calc_bearing_to_point(&pose, &landmark);
    double predicted_bearing = calc_bearing_to_point(&particle, &landmark);
    double likelihood = normal_pdf(measured_bearing, predicted_bearing, 0.1);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 3.9894228, likelihood);
  }
}
