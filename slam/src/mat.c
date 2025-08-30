#include "slam/mat.h"

#include <math.h>

void zero3(float A[9]) {
  for (int i = 0; i < 9; i++) {
    A[i] = 0.0;
  }
}

void mul3(const float A[9], const float B[9], float C[9]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float s = 0.0;
      for (int k = 0; k < 3; k++) {
        s += A[i * 3 + k] * B[k * 3 + j];
      }
      C[i * 3 + j] = s;
    }
  }
}

void transpose3(const float A[9], float At[9]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      At[j * 3 + i] = A[i * 3 + j];
    }
  }
}

void add3(const float A[9], const float B[9], float out[9]) {
  for (int i = 0; i < 9; i++) {
    out[i] = A[i] + B[i];
  }
}

int inv3(const float A[9], float Ainv[9]) {
  float a = A[0], b = A[1], c = A[2];
  float d = A[3], e = A[4], f = A[5];
  float g = A[6], h = A[7], i = A[8];
  float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (fabs(det) < 1e-12)
    return -1;
  float invdet = 1.0 / det;
  Ainv[0] = (e * i - f * h) * invdet;
  Ainv[1] = -(b * i - c * h) * invdet;
  Ainv[2] = (b * f - c * e) * invdet;
  Ainv[3] = -(d * i - f * g) * invdet;
  Ainv[4] = (a * i - c * g) * invdet;
  Ainv[5] = -(a * f - c * d) * invdet;
  Ainv[6] = (d * h - e * g) * invdet;
  Ainv[7] = -(a * h - b * g) * invdet;
  Ainv[8] = (a * e - b * d) * invdet;
  return 0;
}

void symmetrize3(float A[9]) {
  A[1] = A[3] = 0.5 * (A[1] + A[3]);
  A[2] = A[6] = 0.5 * (A[2] + A[6]);
  A[5] = A[7] = 0.5 * (A[5] + A[7]);
}
