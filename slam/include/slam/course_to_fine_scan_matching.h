/*
 * scan_matching.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef SLAM_COURSE_TO_FINE_SCAN_MATCHING_H_
#define SLAM_COURSE_TO_FINE_SCAN_MATCHING_H_

#include <stdbool.h>

#include "map.h"
#include "scan.h"
#include "types.h"

/**
 * @brief Perform scan to map matching on a given scan using a course-to-fine
 * approach. It can converge to a good pose estimate even with a poor initial
 * guess, but is slower than other methods.
 *
 * @details This function performs scan matching by iteratively refining the
 * pose estimate of the robot based on the provided scan data and the occupancy
 * map. It uses the error in the initial guess to determine the search window
 * size and iteratively refines the pose estimate by evaluating the scan match
 * score at various candidate poses. The search is performed in a course-to-fine
 * manner, starting with a larger search window and progressively reducing it
 * as the algorithm converges.
 *
 * It also allows reducing the search space based on the maximum ID of the map
 * cells, which can help in focusing the search on relevant areas of the map
 * (useful, for example, when searching for loop closures against earlier areas
 * of the map).
 *
 * @param scan The scan data to match against the map.
 * @param map The occupancy quadtree map to match the scan against.
 * @param max_id The maximum ID of the map cells to consider for matching.
 * @param initial_guess The initial pose estimate of the robot, which includes
 * @param pose_estimate A pointer to a pose_t structure where the final pose
 * @return true if the scan matching was successful and a valid pose estimate
 * @return false if the scan matching failed or no valid pose estimate could be
 * found.
 */
bool course_to_fine_scan_matching_match(const scan_t *scan,
                                        occupancy_quadtree_t *map,
                                        uint16_t max_id,
                                        const robot_pose_t *initial_guess,
                                        robot_pose_t *pose_estimate);

#endif /* SLAM_SCAN_MATCHING_H_ */
