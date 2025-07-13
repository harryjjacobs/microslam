/*
 * map.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef SLAM_MAP_H_
#define SLAM_MAP_H_

#include "occupancy_quadtree.h"
#include "types.h"
#include "utils.h"

/**
 * @brief Update the map using a scan and robot pose
 *
 * @param map The occupancy quadtree to update
 * @param scan The scan to add to the map
 * @param pose The pose of the robot when the scan was taken
 * @param id Unique identifier for the points added to the map
 * @param weight How much to weight the scan in the update (log odds scale)
 */
void map_add_scan(occupancy_quadtree_t *occupancy, scan_t *scan, pose_t *pose,
                  uint16_t id, int32_t weight);

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
 * @return uint8_t 1 if converged, 0 otherwise
 */
uint8_t map_scan_match(occupancy_quadtree_t *occupancy, scan_t *scan,
                       robot_pose_t *prior, pose_t *estimate, int16_t *score,
                       uint16_t iterations);

/**
 * @brief Calculate the entropy of the map
 *
 * @param occupancy The occupancy quadtree
 * @return float The entropy of the map
 */
float map_entropy(occupancy_quadtree_t *occupancy);

#endif /* SLAM_MAP_H_ */
