/*
 * scan_matching.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef SLAM_SCAN_MATCHING_H_
#define SLAM_SCAN_MATCHING_H_

#include <stdbool.h>

#include "map.h"
#include "scan.h"
#include "types.h"

bool scan_matching_match(const scan_t* current_scan,
                         const lidar_sensor_t* sensor,
                         occupancy_quadtree_t* map, const pose_t* initial_guess,
                         pose_t* pose_estimate, uint16_t max_iterations);

#endif /* SLAM_SCAN_MATCHING_H_ */
