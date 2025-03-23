#include <assert.h>
#include <math.h>
#include <microslam/utils.h>
#include <stdio.h>

float shortest_rotation(float value) {
  if (value < -PI) {
    value += 2 * PI;
  } else if (value > PI) {
    value -= 2 * PI;
  }
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
  return shortest_rotation(value);
}

float random_uniform() { return (float)rand() / RAND_MAX; }

int random_range_uniform(int min, int max) {
  assert(max >= min);
  int range = max - min;
  return min + (rand() % range);
}

float random_range_uniformf(float min, float max) {
  assert(max >= min);
  float range = max - min;
  return min + random_uniform() * range;
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
float random_normal(float mean, float stddev) {
  // Box-Mueller results in two normally distributed random numbers.
  // We can return the second one on the next function call.
  static float next = INFINITY;  // infinity == empty
  if (next != INFINITY) {
    float ret = next;
    next = INFINITY;
    return ret;
  }
  float u1 = random_uniform();
  float u2 = random_uniform();
  float r = sqrt(-2 * log(u1)) * stddev;
  float theta = 2 * PI * u2;
  next = r * sin(theta) + mean;
  return r * cos(theta) + mean;
}

/*
 * Generate a random number in a range using an approximation of the normal
 * distribution.
 */
int random_range_normal(int min, int max) {
  // Central Limit Theorem
  unsigned int N = 32;  // number of averages to take
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
  unsigned int N = 32;  // number of averages to take
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
  static const float inv_sqrt_2pi = 0.3989422804014327;
  float a = (x - mean) / stddev;
  return inv_sqrt_2pi / stddev * exp(-0.5 * a * a);
}

/**
 * @brief Calculate the relative bearing between two points. Not accounting
 * for the rotation of the pose
 *
 * @param pose
 * @param x
 * @param y
 * @return float
 */
float calc_bearing_to_point(pose_t *a, pose_t *b) {
  return rotate((atan2(b->y - a->y, b->x - a->x) - PI / 2), -a->r);
}

void pose_init(pose_t *pose) {
  pose->x = 0;
  pose->y = 0;
  pose->r = 0;
}

float pose_distance(pose_t *a, pose_t *b) {
  return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
}

void pose_add_inplace(pose_t *a, pose_t *b) {
  a->x += b->x;
  a->y += b->y;
  a->r = rotate(a->r, b->r);
}

void pose_add_inplace_unclamped_rot(pose_t *a, pose_t *b) {
  a->x += b->x;
  a->y += b->y;
  a->r += b->r;
}

pose_t pose_subtract(pose_t *a, pose_t *b) {
  pose_t c;
  c.x = a->x - b->x;
  c.y = a->y - b->y;
  c.r = rotate(a->r, -b->r);
  return c;
}

void pose_multiply_inplace_unclamped_rot(pose_t *a, pose_t *b) {
  a->x *= b->x;
  a->y *= b->y;
  a->r *= b->r;
}

void pose_divide_inplace(pose_t *pose, float divisor) {
  pose->x /= divisor;
  pose->y /= divisor;
  pose->r /= divisor;
}
