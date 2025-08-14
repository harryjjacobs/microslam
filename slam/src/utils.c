#include <assert.h>
#include <math.h>
#include <slam/utils.h>
#include <stdio.h>

float wrap_angle(float value) {
  while (value > PI)
    value -= TWO_PI;
  while (value < -PI)
    value += TWO_PI;
  return value;
}

/**
 * @brief Rotate a value by a given rotation
 *
 * @param value
 * @param rotation
 * @return float
 */
float rotate(float value, float rotation) {
  value += rotation;
  return wrap_angle(value);
}

float random_uniformf() { return (float)rand() / RAND_MAX; }

int random_range_uniform(int min, int max) {
  assert(max >= min);
  int range = max - min;
  return min + (rand() % range);
}

float random_range_uniformf(float min, float max) {
  assert(max >= min);
  float range = max - min;
  return min + random_uniformf() * range;
}

/**
 * @brief Generate a random number according to the normal (gaussian)
 * distribution.
 *
 * @details Uses the Box-Muller transform
 * https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
 *
 * @param mean
 * @param stdev
 * @return float
 */
float random_normalf(float mean, float stddev) {
  float u1 = random_uniformf();
  float u2 = random_uniformf();
  float r = sqrtf(-2 * log(u1)) * stddev;
  float theta = 2 * PI * u2;
  return r * cosf(theta) + mean;
}

/*
 * Generate a random number in a range using an approximation of the normal
 * distribution.
 */
int random_range_normal(int min, int max) {
  // Central Limit Theorem
  unsigned int N = 32; // number of averages to take
  int range = max - min;
  long long sum = 0;
  for (unsigned int i = 0; i < N; i++) {
    sum += min + (rand() % range);
  }
  return sum / N;
}

/*
 * Generate a random number in a range using an approximation of the normal
 * distribution.
 */
float random_range_normalf(float min, float max) {
  // Central Limit Theorem
  unsigned int N = 32; // number of averages to take
  long long sum = 0;
  for (unsigned int i = 0; i < N; i++) {
    sum += rand();
  }
  float range = max - min;
  return min + ((float)sum / N) / RAND_MAX * range;
}

/**
 * @brief The probability density function for the normal (gaussian)
 * distribution
 *
 * @param x
 * @param mean
 * @param stddev
 * @return float
 */
float normal_pdf(float x, float mean, float stddev) {
  static const float inv_sqrt_2pi = 0.3989422804014327f;
  float a = (x - mean) / stddev;
  return inv_sqrt_2pi / stddev * expf(-0.5 * a * a);
}

/**
 * @brief Calculate the relative bearing from a to the position of pose b. Not
 * accounting for the rotation of b. It is the rotation that would have to be
 * applied to a to make it point at b.
 *
 * @param a
 * @param b
 * @return float
 */
float calc_bearing_to_point(pose_t *a, pose_t *b) {
  return rotate((atan2f(b->y - a->y, b->x - a->x) - PI_2), -a->r);
}

int16_t euclidean_distance_squared(int16_t x1, int16_t y1, int16_t x2,
                                   int16_t y2) {
  return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

int16_t euclidean_distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
  return sqrt(euclidean_distance_squared(x1, y1, x2, y2));
}

int16_t pose_distance(pose_t *a, pose_t *b) {
  return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
}

uint8_t point_intersects_aabb(int16_t aabb_min_x, int16_t aabb_min_y,
                              int16_t aabb_max_x, int16_t aabb_max_y,
                              int16_t point_x, int16_t point_y) {
  return (point_x >= aabb_min_x && point_x <= aabb_max_x &&
          point_y >= aabb_min_y && point_y <= aabb_max_y);
}

uint8_t ray_intersects_aabb(int16_t aabb_min_x, int16_t aabb_min_y,
                            int16_t aabb_max_x, int16_t aabb_max_y,
                            int16_t ray_origin_x, int16_t ray_origin_y,
                            int16_t ray_direction_x, int16_t ray_direction_y,
                            uint16_t *t_out) {
  float tx1 = ray_direction_x != 0
                  ? (aabb_min_x - ray_origin_x) / ray_direction_x
                  : -INFINITY,
        tx2 = ray_direction_x != 0
                  ? (aabb_max_x - ray_origin_x) / ray_direction_x
                  : INFINITY;
  float ty1 = ray_direction_y != 0
                  ? (aabb_min_y - ray_origin_y) / ray_direction_y
                  : -INFINITY,
        ty2 = ray_direction_y != 0
                  ? (aabb_max_y - ray_origin_y) / ray_direction_y
                  : INFINITY;

  float tmin = fmax(fmin(tx1, tx2), fmin(ty1, ty2));
  float tmax = fmin(fmax(tx1, tx2), fmax(ty1, ty2));

  if (tmax < 0 || tmin > tmax)
    return 0; // No intersection

  // tmin will be negative if the ray starts inside the AABB
  *t_out = (tmin >= 0) ? tmin : tmax;

  return 1;
}

void cholesky_decomp_3x3(float A[3][3], float L[3][3]) {
  // Cholesky decomposition for a 3x3 matrix A
  // A = L * L^T, where L is a lower triangular matrix
  // https://en.wikipedia.org/wiki/Cholesky_decomposition#The_Cholesky%E2%80%93Banachiewicz_and_Cholesky%E2%80%93Crout_algorithms

  L[0][0] = sqrtf(A[0][0]);
  L[1][0] = A[1][0] / L[0][0];
  L[1][1] = sqrtf(A[1][1] - L[1][0] * L[1][0]);
  L[2][0] = A[2][0] / L[0][0];
  L[2][1] = (A[2][1] - L[2][0] * L[1][0]) / L[1][1];
  L[2][2] = sqrtf(A[2][2] - L[2][0] * L[2][0] - L[2][1] * L[2][1]);
}

void solve_linear_system_3x3(float A[3][3], float b[3], float x[3]) {
  // solve A x = -b using cholesky decomposition
  // A is a 3x3 matrix, b is a 3x1 vector, x is the solution vector

  // A = L L^T, where L is a lower triangular matrix
  float L[3][3];
  cholesky_decomp_3x3(A, L);

  // forward substitution to solve L y = -b
  // https://en.wikipedia.org/wiki/Triangular_matrix#Forward_substitution
  float y[3];
  for (int i = 0; i < 3; i++) {
    y[i] = -b[i];
    for (int j = 0; j < i; j++) {
      y[i] -= L[i][j] * y[j];
    }
    y[i] /= L[i][i];
  }

  // backward substitution to solve L^T x = y
  for (int i = 2; i >= 0; i--) {
    x[i] = y[i];
    for (int j = i + 1; j < 3; j++) {
      x[i] -= L[j][i] * x[j];
    }
    x[i] /= L[i][i];
  }
}

float huber_weight(float d, float delta) {
  if (fabsf(d) <= delta)
    return 1.0f; // full weight in quadratic region
  else
    return delta / fabsf(d); // down-weight large errors
}