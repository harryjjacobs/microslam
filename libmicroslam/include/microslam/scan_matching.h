/*
 * scan_matching.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef MICROSLAM_SCAN_MATCHING_H_
#define MICROSLAM_SCAN_MATCHING_H_

#include "map.h"
#include "scan.h"
#include "types.h"

// the scale factor to use for the convergence threshold
#define MICROSLAM_SCAN_MATCHING_CONVERGENCE_EPSILON_FACTOR 0.5f

// how many cells away from the scan point to consider inliers
#define MICROSLAM_SCAN_MATCHING_INLIER_DISTANCE_COUNT 2

/**
 * @brief Perform scan matching using levenberg-marquardt to find the best pose
 * for the scan
 *
 * @param occupancy
 * @param scan
 * @param prior
 * @param estimate
 * @param score
 * @param iterations
 * @return unsigned char 1 if converged, 0 otherwise
 */
unsigned short scan_match_lm(occupancy_quadtree_t *occupancy, scan_t *scan,
                             state_t *prior, pose_t *estimate, float *score,
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
unsigned char scan_match(occupancy_quadtree_t *occupancy, scan_t *scan,
                         state_t *prior, pose_t *estimate, float *score,
                         unsigned short iterations);

#endif /* MICROSLAM_SCAN_MATCHING_H_ */
