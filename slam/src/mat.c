#include "slam/mat.h"

#include <math.h>

void zero6(float A[6][6]) {
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      A[i][j] = 0.0;
}

void zero3(float A[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      A[i][j] = 0.0;
}

void mul6(const float A[6][6], const float B[6][6], float C[6][6]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      float s = 0.0;
      for (int k = 0; k < 6; k++)
        s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
}

void mul6_6_3(const float A[6][6], const float B[6][3], float C[6][3]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      float s = 0.0;
      for (int k = 0; k < 6; k++)
        s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
}

void mul3_6_6(const float A[3][6], const float B[6][6], float C[3][6]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      float s = 0.0;
      for (int k = 0; k < 6; k++)
        s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
}

void mul3_6_3(const float A[3][6], const float B[6][3], float C[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float s = 0.0;
      for (int k = 0; k < 6; k++)
        s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
}

void mul3(const float A[3][3], const float B[3][3], float C[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float s = 0.0;
      for (int k = 0; k < 3; k++)
        s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
}

void transpose6(const float A[6][6], float At[6][6]) {
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      At[j][i] = A[i][j];
}

void transpose6_3(const float A[6][3], float At[3][6]) {
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++)
      At[j][i] = A[i][j];
}

void transpose3_6(const float A[3][6], float At[6][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 6; j++)
      At[j][i] = A[i][j];
}

void add6(float A[6][6], const float B[6][6], float out[6][6]) {
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      out[i][j] = A[i][j] + B[i][j];
}

void add3(float A[3][3], const float B[3][3], float out[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      out[i][j] = A[i][j] + B[i][j];
}

int inv3(const float A[3][3], float Ainv[3][3]) {
  float a = A[0][0], b = A[0][1], c = A[0][2];
  float d = A[1][0], e = A[1][1], f = A[1][2];
  float g = A[2][0], h = A[2][1], i = A[2][2];
  float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (fabs(det) < 1e-12)
    return -1;
  float invdet = 1.0 / det;
  Ainv[0][0] = (e * i - f * h) * invdet;
  Ainv[0][1] = -(b * i - c * h) * invdet;
  Ainv[0][2] = (b * f - c * e) * invdet;
  Ainv[1][0] = -(d * i - f * g) * invdet;
  Ainv[1][1] = (a * i - c * g) * invdet;
  Ainv[1][2] = -(a * f - c * d) * invdet;
  Ainv[2][0] = (d * h - e * g) * invdet;
  Ainv[2][1] = -(a * h - b * g) * invdet;
  Ainv[2][2] = (a * e - b * d) * invdet;
  return 0;
}
