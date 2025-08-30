#ifndef SLAM_EKF_H_
#define SLAM_EKF_H_

#include <slam/types.h>

typedef struct {
  float x[3]; // state vector (x, y, theta)
  float P[9]; // covariance
} ekf_t;

/**
 * @brief Initialize the EKF state
 * @param ekf The EKF state
 * @param x0 The initial state vector (x, y, theta)
 * @param P0 The initial covariance matrix, 3x3 row-major
 */
void ekf_init(ekf_t *ekf, const float x0[3], const float P0[9]);

/**
 * @brief Prediction step (in-place)
 * @details Motion model (u in robot frame):
 *   x' = x + R(th) * [dx; dy]
 *   th' = th + dth
 *
 * @param ekf The EKF state
 * @param u The control input (dx, dy, dth)
 * @param Q The process noise covariance, 3x3 row-major
 */
void ekf_predict(ekf_t *ekf, const float u[3], const float Q[9]);

/**
 * @brief Update step with absolute pose measurement z = [x, y, theta]
 *
 * @param ekf The EKF state
 * @param z The measurement (x, y, theta)
 * @param R The measurement noise covariance, 3x3 row-major
 */
int ekf_update_pose(ekf_t *ekf, const float z[3], const float R[9]);

/**
 * @brief Get the current state vector
 * @param ekf The EKF state
 * @return Pointer to the state vector (x, y, theta)
 */
const float *ekf_state(const ekf_t *ekf);

/**
 * @brief Get the current covariance matrix
 * @param ekf The EKF state
 * @return Pointer to the covariance matrix, 3x3 row-major
 */
const float *ekf_cov(const ekf_t *ekf);

#endif // SLAM_EKF_H_
