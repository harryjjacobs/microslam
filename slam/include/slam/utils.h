/*
 * utils.h
 *
 *  Created on: 13 Mar 2024
 *      Author: harryjjacobs
 */

#ifndef INCLUDE_SLAM_UTILS_H_
#define INCLUDE_SLAM_UTILS_H_

#include <slam/types.h>
#include <stdlib.h>

#define PI 3.14159265359f
#define TWO_PI 6.28318530718f
#define PI_2 1.57079632679f

#define DEG2RAD(deg) ((deg) * PI / 180.0f)
#define RAD2DEG(rad) ((rad) * 180.0f / PI)

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(value, min, max)                                                 \
  ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

/**
 * @brief Clamp a rotation to the range -PI to PI
 *
 * @param value
 * @return float
 */
float wrap_angle(float value);

/**
 * @brief Rotate a value by a given rotation
 *
 * @param value
 * @param rotation
 * @return float
 */
float rotate(float value, float rotation);

/**
 * @brief Generate a random number from a uniform distribution between 0 and 1
 *
 * @return float
 */
float random_uniformf();

/**
 * @brief Generate a random integer number in a range using a uniform
 * distribution
 *
 * @param min
 * @param max
 * @return int
 */
int random_range_uniform(int min, int max);

/**
 * @brief Generate a random floating point number in a range using a uniform
 * distribution
 *
 * @param min
 * @param max
 * @return float
 */
float random_range_uniformf(float min, float max);

/**
 *
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
float random_normalf(float mean, float stddev);
/*
 * Generate a random number in a range using an approximation of the normal
 * distribution.
 */
int random_range_normal(int min, int max);

/*
 * Generate a random number in a range using an approximation of the normal
 * distribution.
 */
float random_range_normalf(float min, float max);

/**
 * @brief The probability density function for the normal (gaussian)
 * distribution
 *
 * @param x
 * @param mean
 * @param stddev
 * @return float
 */
float normal_pdf(float x, float mean, float stddev);

/**
 * @brief Calculate the relative bearing from a pose to a point.
 *
 * i.e. What is the rotation I would have to apply to make the pose face
 * the point at x, y
 *
 * @param pose
 * @param x
 * @param y
 * @return float
 */
float calc_bearing_to_point(pose_t *a, pose_t *b);

/**
 * @brief Calculate the squared euclidean distance squared between two points
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return
 */
int16_t euclidean_distance_squared(int16_t x1, int16_t y1, int16_t x2,
                                   int16_t y2);

/**
 * @brief Calculate the euclidean distance between two poses
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return
 */
int16_t euclidean_distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

/**
 * @brief Calculate the euclidean distance between two poses
 *
 * @param a
 * @param b
 * @return float
 */
int16_t pose_distance(pose_t *a, pose_t *b);

/**
 * @brief Check if a point intersects an axis-aligned bounding box (AABB)
 *
 * @param aabb_min_x
 * @param aabb_min_y
 * @param aabb_max_x
 * @param aabb_max_y
 * @param point_x
 * @param point_y
 * @return uint8_t
 */
uint8_t point_intersects_aabb(int16_t aabb_min_x, int16_t aabb_min_y,
                              int16_t aabb_max_x, int16_t aabb_max_y,
                              int16_t point_x, int16_t point_y);

/**
 * @brief Check if a ray intersects an axis-aligned bounding box
 *
 * @param aabb_min_x The minimum x coordinate of the AABB
 * @param aabb_min_y The minimum y coordinate of the AABB
 * @param aabb_max_x The maximum x coordinate of the AABB
 * @param aabb_max_y The maximum y coordinate of the AABB
 * @param ray_origin_x The x coordinate of the ray origin
 * @param ray_origin_y The y coordinate of the ray origin
 * @param ray_direction_x The x component of the ray direction
 * @param ray_direction_y The y component of the ray direction
 * @param t_out The distance along the ray where the nearest intersection occurs
 * @return uint8_t
 */
uint8_t ray_intersects_aabb(int16_t aabb_min_x, int16_t aabb_min_y,
                            int16_t aabb_max_x, int16_t aabb_max_y,
                            int16_t ray_origin_x, int16_t ray_origin_y,
                            int16_t ray_direction_x, int16_t ray_direction_y,
                            uint16_t *t_out);

/**
 * @brief Calculate the Cholesky decomposition of a 3x3 matrix
 *
 * @param A
 * @param L
 */
void cholesky_decomp_3x3(float A[3][3], float L[3][3]);

/**
 * @brief Solve a linear system of equations Ax = b using Cholesky decomposition
 *
 * @param A
 * @param b
 * @param x
 */
void solve_linear_system_3x3(float A[3][3], float b[3], float x[3]);

/**
 * @brief Calculate the Huber weight for a given distance and delta.
 * Behaves quadratically for small distances and linearly for large ones.
 *
 * @param d
 * @param delta
 * @return float
 */
float huber_weight(float d, float delta);

#endif /* INCLUDE_SLAM_UTILS_H_ */
