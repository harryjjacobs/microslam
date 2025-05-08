#include <assert.h>
#include <math.h>
#include <microslam/utils.h>
#include <stdio.h>

float clamp_rotation(float value) {
  while (value > PI) value -= TWO_PI;
  while (value < -PI) value += TWO_PI;
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
  return clamp_rotation(value);
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
float random_normalf(float mean, float stddev) {
  float u1 = random_uniform();
  float u2 = random_uniform();
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

float euclidean_distance_squared(float x1, float y1, float x2, float y2) {
  return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

float euclidean_distance(float x1, float y1, float x2, float y2) {
  return sqrtf(euclidean_distance_squared(x1, y1, x2, y2));
}

void pose_init(pose_t *pose) {
  pose->x = 0;
  pose->y = 0;
  pose->r = 0;
}

float pose_distance(pose_t *a, pose_t *b) {
  return sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
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

unsigned char ray_intersects_aabb(float aabb_min_x, float aabb_min_y,
                                  float aabb_max_x, float aabb_max_y,
                                  float ray_origin_x, float ray_origin_y,
                                  float ray_direction_x, float ray_direction_y,
                                  float *t_out) {
  float tx1 = (aabb_min_x - ray_origin_x) / ray_direction_x,
        tx2 = (aabb_max_x - ray_origin_x) / ray_direction_x;
  float ty1 = (aabb_min_y - ray_origin_y) / ray_direction_y,
        ty2 = (aabb_max_y - ray_origin_y) / ray_direction_y;

  float tmin = fmax(fmin(tx1, tx2), fmin(ty1, ty2));
  float tmax = fmin(fmax(tx1, tx2), fmax(ty1, ty2));

  if (tmax < 0 || tmin > tmax) return 0;  // No intersection

  // tmin will be negative if the ray starts inside the AABB
  *t_out = (tmin >= 0) ? tmin : tmax;

  return 1;
}