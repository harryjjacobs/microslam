#ifndef SLAM_EKF_H_
#define SLAM_EKF_H_

#include <slam/types.h>

#define SLAM_EKF_STATE_N 6
#define SLAM_EKF_MEAS_N 3

typedef struct {
  float x[SLAM_EKF_STATE_N];                   // state vector
  float P[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N]; // covariance
  float Q[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N]; // process noise covariance
} ekf_t;

// initialize EKF with initial state x0, covariance P0, and process noise
// parameters
void ekf_init(
    ekf_t *kf, const float x0[SLAM_EKF_STATE_N],
    const float P0[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N],
    float sigma_pos,   // heuristic stdev for position process noise (m)
    float sigma_vel,   // stdev for velocity process noise (m/s)
    float sigma_theta, // stdev for heading process noise (rad)
    float sigma_omega  // stdev for angular velocity process noise (rad/s)
);

// predict step with time delta dt
void ekf_predict(ekf_t *kf, float dt);

// update step with measurement z and measurement covariance R
int ekf_update(ekf_t *kf, const float z[SLAM_EKF_MEAS_N],
               const float R[SLAM_EKF_MEAS_N][SLAM_EKF_MEAS_N]);

// retrieve state
void ekf_get_state(const ekf_t *kf, float out_x[SLAM_EKF_STATE_N],
                   float out_P[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N]);

#endif // SLAM_EKF_H_
