/*
 * microslam.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef MICROSLAM_MAP_H_
#define MICROSLAM_MAP_H_

#include "occupancy_quadtree.h"
#include "types.h"
#include "utils.h"

/**
 * @brief Update the map using a scan and robot pose
 *
 * @param map
 * @param scan
 * @param pose
 */
void map_add_scan(occupancy_quadtree_t *occupancy, scan_t *scan, pose_t *pose);

/**
 * @brief Compute the score of a scan at a given pose
 *
 * @param occupancy
 * @param scan
 * @param pose
 * @param score
 */
void compute_scan_score(occupancy_quadtree_t *occupancy, scan_t *scan,
                        pose_t *pose, float *score);

void map_scan_match_gradient(occupancy_quadtree_t *occupancy, scan_t *scan,
                             pose_t *pose, pose_t *gradient, float *score);

/**
 * @brief Perform scan matching to find the best pose for the scan
 *
 * @param occupancy
 * @param scan
 * @param prior
 * @param estimate
 * @param score
 * @param iterations
 * @return unsigned char 1 if converged, 0 otherwise
 */
unsigned char map_scan_match(occupancy_quadtree_t *occupancy, scan_t *scan,
                             state_t *prior, pose_t *estimate, float *score,
                             unsigned short iterations);

/**
 * @brief Calculate the entropy of the map
 *
 * @param occupancy The occupancy quadtree
 * @return float The entropy of the map
 */
float map_entropy(occupancy_quadtree_t *occupancy);

#endif /* MICROSLAM_MAP_H_ */
