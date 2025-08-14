#include "slam/ekf.h"
#include "slam/mat.h"

#include <slam/utils.h>

#include <math.h>
#include <slam/logging.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// utility: zero matrix

// initialize ekf_t
void ekf_init(
    ekf_t *kf, const float x0[SLAM_EKF_STATE_N],
    const float P0[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N],
    float sigma_pos,   // heuristic stdev for position process noise (m)
    float sigma_vel,   // stdev for velocity process noise (m/s)
    float sigma_theta, // stdev for heading process noise (rad)
    float sigma_omega  // stdev for angular velocity process noise (rad/s)
) {
  memcpy(kf->x, x0, sizeof(float) * SLAM_EKF_STATE_N);
  for (int i = 0; i < SLAM_EKF_STATE_N; i++)
    for (int j = 0; j < SLAM_EKF_STATE_N; j++)
      kf->P[i][j] = P0[i][j];

  // Simple diag Q: let user provide approximate stddevs.
  zero6(kf->Q);
  // position
  kf->Q[0][0] = sigma_pos * sigma_pos;
  kf->Q[1][1] = sigma_pos * sigma_pos;
  // heading
  kf->Q[2][2] = sigma_theta * sigma_theta;
  // velocities
  kf->Q[3][3] = sigma_vel * sigma_vel;
  kf->Q[4][4] = sigma_vel * sigma_vel;
  kf->Q[5][5] = sigma_omega * sigma_omega;
}

// small helper: C = A*B where A and B are 6x6

// predict step: dt in seconds
void ekf_predict(ekf_t *kf, float dt) {
  // state transition f(x)
  float px = kf->x[0];
  float py = kf->x[1];
  float theta = kf->x[2];
  float vx = kf->x[3];
  float vy = kf->x[4];
  float omega = kf->x[5];

  // simple constant velocity in world frame
  float px_p = px + vx * dt;
  float py_p = py + vy * dt;
  float theta_p = wrap_angle(theta + omega * dt);
  float vx_p = vx;
  float vy_p = vy;
  float omega_p = omega;

  kf->x[0] = px_p;
  kf->x[1] = py_p;
  kf->x[2] = theta_p;
  kf->x[3] = vx_p;
  kf->x[4] = vy_p;
  kf->x[5] = omega_p;

  // Jacobian F (6x6): identity with dt at (0,3), (1,4), (2,5)
  float F[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N];
  zero6(F);
  for (int i = 0; i < SLAM_EKF_STATE_N; i++)
    F[i][i] = 1.0;
  F[0][3] = dt;
  F[1][4] = dt;
  F[2][5] = dt;

  // P = F*P*F^T + Q
  float FP[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N];
  mul6(F, kf->P, FP);
  float Ft[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N];
  transpose6(F, Ft);
  float FPFt[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N];
  mul6(FP, Ft, FPFt);
  float Pnew[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N];
  add6(FPFt, kf->Q, Pnew);

  memcpy(kf->P, Pnew, sizeof(kf->P));
}

// update with measurement z = [px, py, theta], R is 3x3 measurement covariance
// returns 0 on success, -1 on singular S
int ekf_update(ekf_t *kf, const float z[SLAM_EKF_MEAS_N],
               const float R[SLAM_EKF_MEAS_N][SLAM_EKF_MEAS_N]) {
  // measurement prediction h(x) = [px, py, theta]
  float hx[SLAM_EKF_MEAS_N];
  hx[0] = kf->x[0];
  hx[1] = kf->x[1];
  hx[2] = kf->x[2];

  // innovation y = z - h, normalize angle
  float y[SLAM_EKF_MEAS_N];
  y[0] = z[0] - hx[0];
  y[1] = z[1] - hx[1];
  y[2] = wrap_angle(z[2] - hx[2]);

  // H matrix 3x6
  float H[SLAM_EKF_MEAS_N][SLAM_EKF_STATE_N] = {0};
  H[0][0] = 1.0;
  H[1][1] = 1.0;
  H[2][2] = 1.0;

  float Ht[SLAM_EKF_STATE_N][SLAM_EKF_MEAS_N];
  transpose3_6(H, Ht); // H^T (6x3)

  // S = H * P * H^T + R  (3x3)
  float HP[SLAM_EKF_MEAS_N][SLAM_EKF_STATE_N];
  // compute HP = H * P  (3x6)
  for (int i = 0; i < SLAM_EKF_MEAS_N; i++) {
    for (int j = 0; j < SLAM_EKF_STATE_N; j++) {
      float s = 0.0;
      for (int k = 0; k < SLAM_EKF_STATE_N; k++)
        s += H[i][k] * kf->P[k][j];
      HP[i][j] = s;
    }
  }
  float S[SLAM_EKF_MEAS_N][SLAM_EKF_MEAS_N];
  mul3_6_3(HP, Ht, S); // S = H P H^T
  add3(S, R, S);       // S = S + R

  // invert S
  float Sinv[SLAM_EKF_MEAS_N][SLAM_EKF_MEAS_N];
  if (inv3(S, Sinv) != 0)
    return -1;

  // K = P * H^T * S^{-1}  (6x3)
  float PHt[SLAM_EKF_STATE_N][SLAM_EKF_MEAS_N];
  mul6_6_3(kf->P, Ht, PHt); // (6x6)*(6x3) -> 6x3
  // multiply PHt (6x3) by Sinv (3x3) => K (6x3)
  float K[SLAM_EKF_STATE_N][SLAM_EKF_MEAS_N];
  for (int i = 0; i < SLAM_EKF_STATE_N; i++) {
    for (int j = 0; j < SLAM_EKF_MEAS_N; j++) {
      float s = 0.0;
      for (int k = 0; k < SLAM_EKF_MEAS_N; k++)
        s += PHt[i][k] * Sinv[k][j];
      K[i][j] = s;
    }
  }

  // state update x = x + K*y
  for (int i = 0; i < SLAM_EKF_STATE_N; i++) {
    float s = 0.0;
    for (int j = 0; j < SLAM_EKF_MEAS_N; j++)
      s += K[i][j] * y[j];
    kf->x[i] += s;
  }
  // normalize heading
  kf->x[2] = wrap_angle(kf->x[2]);

  // P = (I - K H) P
  float KH[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N];
  zero6(KH);
  for (int i = 0; i < SLAM_EKF_STATE_N; i++) {
    for (int j = 0; j < SLAM_EKF_STATE_N; j++) {
      float s = 0.0;
      for (int k = 0; k < SLAM_EKF_MEAS_N; k++)
        s += K[i][k] * H[k][j];
      KH[i][j] = s;
    }
  }
  float IminusKH[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N];
  for (int i = 0; i < SLAM_EKF_STATE_N; i++) {
    for (int j = 0; j < SLAM_EKF_STATE_N; j++) {
      IminusKH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
    }
  }
  float Pnew[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N];
  mul6(IminusKH, kf->P, Pnew);
  memcpy(kf->P, Pnew, sizeof(kf->P));

  return 0;
}

// retrieve state
void ekf_get_state(const ekf_t *kf, float out_x[SLAM_EKF_STATE_N],
                   float out_P[SLAM_EKF_STATE_N][SLAM_EKF_STATE_N]) {
  if (out_x != NULL) {
    memcpy(out_x, kf->x, sizeof(float) * SLAM_EKF_STATE_N);
  }
  if (out_P != NULL) {
    memcpy(out_P, kf->P, sizeof(float) * SLAM_EKF_STATE_N * SLAM_EKF_STATE_N);
  }
}
