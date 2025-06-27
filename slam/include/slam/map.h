/*
 * map.h
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
 * @param weight How much to weight the scan in the update (log odds scale)
 */
void map_add_scan(occupancy_quadtree_t *occupancy, scan_t *scan, pose_t *pose,
                  float weight);

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
unsigned short map_scan_match_lm(occupancy_quadtree_t *occupancy, scan_t *scan,
                                 robot_pose_t *prior, pose_t *estimate,
                                 float *score, unsigned short iterations);

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
unsigned char map_scan_match(occupancy_quadtree_t *occupancy, scan_t *scan,
                             robot_pose_t *prior, pose_t *estimate,
                             float *score, unsigned short iterations);

/**
 * @brief Calculate the entropy of the map
 *
 * @param occupancy The occupancy quadtree
 * @return float The entropy of the map
 */
float map_entropy(occupancy_quadtree_t *occupancy);

#endif /* MICROSLAM_MAP_H_ */
