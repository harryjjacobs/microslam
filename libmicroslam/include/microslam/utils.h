/*
 * utils.h
 *
 *  Created on: 13 Mar 2023
 *      Author: harryjjacobs
 */

#ifndef INCLUDE_MICROSLAM_UTILS_H_
#define INCLUDE_MICROSLAM_UTILS_H_

#include <microslam/types.h>
#include <stdlib.h>

#define PI 3.14159265359

double shortest_rotation(double value);

/**
 * @brief Rotate a value by a given rotation
 *
 * @param value
 * @param rotation
 * @return double
 */
double rotate(double value, double rotation);

double random_uniform();

int random_range_uniform(int min, int max);

double random_range_uniformf(double min, double max);

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
 * @return double
 */
double random_normal(double mean, double stddev);
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
 * @return double
 */
double normal_pdf(double x, double mean, double stddev);

/**
 * @brief Calculate the relative bearing from a pose to a point.
 *
 * i.e. What is the rotation I would have to apply to make the pose face
 * the point at x, y
 *
 * @param pose
 * @param x
 * @param y
 * @return double
 */
double calc_bearing_to_point(pose_t *a, pose_t *b);

/**
 * @brief Sets all components of a pose to zero
 *
 * @param pose
 */
void pose_init(pose_t *pose);

double pose_distance(pose_t *a, pose_t *b);

void pose_add_inplace(pose_t *a, pose_t *b);

void pose_add_inplace_unclamped_rot(pose_t *a, pose_t *b);

pose_t pose_subtract(pose_t *a, pose_t *b);

void pose_multiply_inplace_unclamped_rot(pose_t *a, pose_t *b);

void pose_divide_inplace(pose_t *pose, double divisor);

#endif /* INCLUDE_MICROSLAM_UTILS_H_ */
