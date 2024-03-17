/*
 * microslam.h
 *
 *  Created on: Mar 5, 2023
 *      Author: harryjjacobs
 */

#ifndef MICROSLAM_MAP_H_
#define MICROSLAM_MAP_H_

#include "types.h"
#include "utils.h"

void map_init(map_t *map);

void map_add_landmark(map_t *map, pose_t pose, int signature);

void map_destroy(map_t *map);

#endif /* MICROSLAM_MAP_H_ */
