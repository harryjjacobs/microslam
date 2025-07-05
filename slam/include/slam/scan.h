/*
 * slam.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef SLAM_SCAN_H_
#define SLAM_SCAN_H_

#include <stdio.h>
#include <stdlib.h>

#include "types.h"

void scan_init(scan_t *scan, float bearing_error, float range_error);

void scan_reset(scan_t *scan);

void scan_add(scan_t *scan, unsigned short bearing, float range);

// void scan_add(scan_t *scan, observation_t observation);

#endif /* SLAM_SCAN_H_ */
