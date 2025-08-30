#include "slam/ekf.h"
#include "slam/mat.h"

#include <slam/utils.h>

#include <math.h>
#include <slam/logging.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void ekf_init(ekf_t *ekf, const float x0[3], const float P0[9]) {
  for (int i = 0; i < 3; i++) {
    ekf->x[i] = x0[i];
  }
  for (int i = 0; i < 9; i++) {
    ekf->P[i] = P0[i];
  }
  ekf->x[2] = wrap_angle(ekf->x[2]);
}

void ekf_predict(ekf_t *ekf, const float u[3], const float Q[9]) {
  float th = ekf->x[2];
  float c = cos(th), s = sin(th);
  float dx = u[0], dy = u[1], dth = u[2];

  // In-place state update
  ekf->x[0] += c * dx - s * dy;
  ekf->x[1] += s * dx + c * dy;
  ekf->x[2] = wrap_angle(th + dth);

  // Jacobians
  float F[9] = {1.0, 0.0, -s * dx - c * dy, 0.0, 1.0, c * dx - s * dy, 0.0,
                0.0, 1.0};
  float G[9] = {c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0};

  // P = F P F^T + G Ru G^T
  float FP[9], FT[9], FPFt[9];
  float GRu[9], GT[9], GRuGTt[9];

  mul3(F, ekf->P, FP);
  transpose3(F, FT);
  mul3(FP, FT, FPFt);

  mul3(G, Q, GRu);
  transpose3(G, GT);
  mul3(GRu, GT, GRuGTt);

  for (int i = 0; i < 9; i++) {
    ekf->P[i] = FPFt[i] + GRuGTt[i];
  }
  symmetrize3(ekf->P);
}

int ekf_update_pose(ekf_t *ekf, const float z[3], const float R[9]) {
  float nu[3];
  nu[0] = z[0] - ekf->x[0];
  nu[1] = z[1] - ekf->x[1];
  nu[2] = wrap_angle(z[2] - ekf->x[2]);

  // S = P + Rz
  float S[9];
  for (int i = 0; i < 9; i++)
    S[i] = ekf->P[i] + ekf->Rz[i];

  // Invert S
  float Sinv[9];
  if (inv3(S, Sinv) != 0)
    return -1;

  // K = P * Sinv
  float K[9];
  mul3(ekf->P, Sinv, K);

  // delta = K * nu  (3x3 * 3x1)
  float delta[3];
  delta[0] = K[0] * nu[0] + K[1] * nu[1] + K[2] * nu[2];
  delta[1] = K[3] * nu[0] + K[4] * nu[1] + K[5] * nu[2];
  delta[2] = K[6] * nu[0] + K[7] * nu[1] + K[8] * nu[2];

  // State update (in place)
  ekf->x[0] += delta[0];
  ekf->x[1] += delta[1];
  ekf->x[2] = wrap_angle(ekf->x[2] + delta[2]);

  /* Joseph cov update:
   * 1) compute IK = I - K
   * 2) T = IK * P
   * 3) P_left = T * IK^T
   * 4) KR = K * Rz
   * 5) P_right = KR * K^T
   * 6) P = P_left + P_right
   *
   * Temporaries:
   *   IK  (9)  - I minus K
   *   T   (9)  - IK * P
   *   Pout(9)  - final P (then copied into ekf->P)
   *   KR  (9)  - K * Rz
   *
   */

  float IK[9];
  IK[0] = 1.0 - K[0];
  IK[1] = -K[1];
  IK[2] = -K[2];
  IK[3] = -K[3];
  IK[4] = 1.0 - K[4];
  IK[5] = -K[5];
  IK[6] = -K[6];
  IK[7] = -K[7];
  IK[8] = 1.0 - K[8];

  float T[9];
  mul3(IK, ekf->P, T);

  float IKt[9];
  transpose3(IK, IKt);

  float Pleft[9];
  mul3(T, IKt, Pleft); // Pleft = (I-K) P (I-K)^T

  float KR[9];
  mul3(K, ekf->Rz, KR); // KR = K * Rz

  float Kt[9];
  transpose3(K, Kt);

  float Pright[9];
  mul3(KR, Kt, Pright); // Pright = K Rz K^T

  // Pout = Pleft + Pright
  for (int i = 0; i < 9; i++)
    ekf->P[i] = Pleft[i] + Pright[i];

  // Ensure symmetry
  symmetrize3(ekf->P);
  return 0;
}

const float *ekf_state(const ekf_t *ekf) { return ekf->x; }

const float *ekf_cov(const ekf_t *ekf) { return ekf->P; }
