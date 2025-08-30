#ifndef SLAM_ODOMETRY_H_
#define SLAM_ODOMETRY_H_

#include <slam/types.h>

typedef struct odometry_t {
  float dx;
  float dy;
  float dr;
  float covariance[3][3];
} odometry_t;

void slam_odometry_reset_pose(odometry_t *odometry);

void slam_odometry_reset_covariance(odometry_t *odometry);

void slam_odometry_covariance_diagonal(odometry_t *odometry, float var_xx,
                                       float var_yy, float var_rr);

#endif // SLAM_ODOMETRY_H_
