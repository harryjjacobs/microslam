#include "slam/ekf.h"

#include <math.h>
#include <slam/utils.h>
#include <string.h>
#include <unity/unity.h>

#define TOL 1e-6

ekf_t kf;

void setUp(void) {
  // initial state: px=0, py=0, theta=0, vx=1, vy=0.5, omega=0.1
  float x0[SLAM_EKF_STATE_N] = {0, 0, 0, 1.0, 0.5, 0.1};
  float P0[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N] = {0};
  for (int i = 0; i < SLAM_EKF_STATE_N; i++)
    P0[i][i] = 1e-3;

  ekf_init(&kf, x0, P0, 1e-3, 1e-3, 1e-4, 1e-5);
}

void tearDown(void) {
  // nothing to do
}

void test_initialization(void) {
  float x[SLAM_EKF_STATE_N];
  float P[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N];
  ekf_get_state(&kf, x, P);

  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0, x[0]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0, x[1]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0, x[2]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0, x[3]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.5, x[4]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.1, x[5]);

  for (int i = 0; i < SLAM_EKF_STATE_N; i++) {
    for (int j = 0; j < SLAM_EKF_STATE_N; j++) {
      if (i == j) {
        TEST_ASSERT_FLOAT_WITHIN(TOL, 1e-3, P[i][j]);
      } else {
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0, P[i][j]);
      }
    }
  }
}

void test_prediction(void) {
  float dt = 0.1;
  ekf_predict(&kf, dt);

  float x[SLAM_EKF_STATE_N];
  ekf_get_state(&kf, x, (float (*)[SLAM_EKF_STATE_N])NULL);

  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0 + 1.0 * dt, x[0]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0 + 0.5 * dt, x[1]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0 + 0.1 * dt, x[2]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0, x[3]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.5, x[4]);
  TEST_ASSERT_FLOAT_WITHIN(TOL, 0.1, x[5]);
}

void test_update_simple_measurement(void) {
  float dt = 0.1;
  ekf_predict(&kf, dt);

  // measurement close to predicted (+0.01, +0.001, +0.001)
  float z[SLAM_EKF_MEAS_N] = {0.11, 0.051, 0.011};
  float R[SLAM_EKF_MEAS_N][SLAM_EKF_MEAS_N] = {0};
  R[0][0] = 1e-2;
  R[1][1] = 1e-2;
  R[2][2] = 1e-3;

  int status = ekf_update(&kf, z, R);
  TEST_ASSERT_EQUAL_INT(0, status);

  float x[SLAM_EKF_STATE_N];
  ekf_get_state(&kf, x, (float (*)[SLAM_EKF_STATE_N])NULL);

  // The state should be closer to the measurement than the prior
  TEST_ASSERT_LESS_THAN_FLOAT(fabs(0.0 + 1.0 * dt - z[0]), fabs(x[0] - z[0]));
  TEST_ASSERT_LESS_THAN_FLOAT(fabs(0.0 + 0.5 * dt - z[1]), fabs(x[1] - z[1]));
  TEST_ASSERT_LESS_THAN_FLOAT(fabs(wrap_angle(0.0 + 0.1 * dt - z[2])),
                              fabs(wrap_angle(x[2] - z[2])));
}

void test_predict_update_loop(void) {
  float dt = 0.1;
  for (int i = 0; i < 10; i++) {
    ekf_predict(&kf, dt);

    // simulate a measurement that is close to the predicted state
    float z[SLAM_EKF_MEAS_N];
    z[0] = kf.x[0] + random_normalf(0, 1e-2);
    z[1] = kf.x[1] + random_normalf(0, 1e-2);
    z[2] = wrap_angle(kf.x[2] + random_normalf(0, 1e-3));

    float R[SLAM_EKF_MEAS_N][SLAM_EKF_MEAS_N] = {0};
    R[0][0] = 1e-2;
    R[1][1] = 1e-2;
    R[2][2] = 1e-3;

    int status = ekf_update(&kf, z, R);
    TEST_ASSERT_EQUAL_INT(0, status);

    float x[SLAM_EKF_STATE_N];
    ekf_get_state(&kf, x, (float (*)[SLAM_EKF_STATE_N])NULL);

    // Check that the state is updated towards the measurement
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(fabs(kf.x[0] - z[0]), fabs(x[0] - z[0]));
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(fabs(kf.x[1] - z[1]), fabs(x[1] - z[1]));
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(fabs(wrap_angle(kf.x[2] - z[2])),
                                    fabs(wrap_angle(x[2] - z[2])));
  }
}

void test_covariance_positive(void) {
  float dt = 0.1;
  ekf_predict(&kf, dt);

  float z[SLAM_EKF_MEAS_N] = {0.2, 0.1, 0.05};
  float R[SLAM_EKF_MEAS_N][SLAM_EKF_MEAS_N] = {0};
  R[0][0] = 1e-4;
  R[1][1] = 1e-4;
  R[2][2] = 1e-5;
  ekf_update(&kf, z, R);

  // basic check: diagonal elements > 0
  for (int i = 0; i < SLAM_EKF_STATE_N; i++) {
    TEST_ASSERT_TRUE(kf.P[i][i] > 0.0);
  }
}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_initialization);
  RUN_TEST(test_prediction);
  RUN_TEST(test_update_simple_measurement);
  RUN_TEST(test_predict_update_loop);
  RUN_TEST(test_covariance_positive);

  return UNITY_END();
}
