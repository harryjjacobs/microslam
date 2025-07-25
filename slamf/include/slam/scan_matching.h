/*
 * scan_matching.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef SLAM_SCAN_MATCHING_H_
#define SLAM_SCAN_MATCHING_H_

#include "map.h"
#include "scan.h"
#include "types.h"

// the scale factor to use for the convergence threshold
#define SLAM_SCAN_MATCHING_CONVERGENCE_EPSILON_FACTOR 0.1f

// how many cells away from the scan point to consider inliers
#define SLAM_SCAN_MATCHING_INLIER_DISTANCE_COUNT 5

/**
 * @brief Compute the gradient of the scan matching cost function.
 * The cost function is the sum of the squared distances to the nearest leaf:
 * E(p)=∑_i(d(T_p(s_i)))^2
 *
 * where p is the pose, s_i is the scan point, T_p(s_i) is the transformed
 * scan point s_i in world space and d is the distance to the nearest leaf in
 * the quadtree.
 *
 * The gradient is computed using finite differences, and the sum of the
 * squared distances is returned in the sum parameter.
 *
 * The gradient can be thought of as the direction in which the cost
 * function decreases the fastest, and the magnitude of the gradient
 * indicates how steep the cost function is in that direction.
 *
 * @param occupancy The occupancy quadtree of the map
 * @param scan The scan to match to the map
 * @param pose The pose of the robot in world space when the scan was taken
 * @param gradient The gradient of the cost function w.r.t. the pose
 * @param sum The sum of the squared distances to the nearest leaf
 *
 * @return unsigned short 1 if the gradient was computed successfully,
 * 0 if not enough matches were found.
 */
unsigned short scan_matching_gradient(occupancy_quadtree_t *occupancy,
                                      scan_t *scan, pose_t *pose,
                                      float min_matches, pose_t *gradient,
                                      float *sum);

/**
 * @brief Perform scan matching using levenberg-marquardt to find the best pose
 * for the scan
 *
 * @param occupancy The occupancy quadtree of the map
 * @param scan The scan to match to the map
 * @param prior The prior state of the robot
 * @param estimate The estimated pose of the robot after scan matching
 * @param score The score of the scan matching, i.e. the sum of the squared
 * @param iterations The maximum number of iterations to perform
 * @return unsigned char 1 if converged, 0 otherwise
 */
unsigned short scan_matching_match_lm(occupancy_quadtree_t *occupancy,
                                      scan_t *scan, pose_t *prior,
                                      pose_t *estimate, float *score,
                                      unsigned short iterations);

/**
 * @brief Perform scan matching using gradient descent to find the best pose for
 * the scan
 *
 * @param occupancy
 * @param scan
 * @param prior
 * @param estimate
 * @param score
 * @param iterations
 * @return unsigned char 1 if converged, 0 otherwise
 */
unsigned char scan_matching_match(occupancy_quadtree_t *occupancy, scan_t *scan,
                                  pose_t *prior, pose_t *estimate, float *score,
                                  unsigned short iterations);

#endif /* SLAM_SCAN_MATCHING_H_ */
