#include <slam/odometry.h>

void slam_odometry_reset_pose(odometry_t *odometry) {
  odometry->dx = 0.0f;
  odometry->dy = 0.0f;
  odometry->dr = 0.0f;
}

void slam_odometry_reset_covariance(odometry_t *odometry) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      odometry->covariance[i][j] = (i == j) ? 1.0f : 0.0f; // Identity matrix
    }
  }
}

void slam_odometry_covariance_diagonal(odometry_t *odometry, float var_xx,
                                       float var_yy, float var_rr) {
  odometry->covariance[0][0] = var_xx;
  odometry->covariance[0][1] = 0.0f;
  odometry->covariance[0][2] = 0.0f;
  odometry->covariance[1][0] = 0.0f;
  odometry->covariance[1][1] = var_yy;
  odometry->covariance[1][2] = 0.0f;
  odometry->covariance[2][0] = 0.0f;
  odometry->covariance[2][1] = 0.0f;
  odometry->covariance[2][2] = var_rr;
}
