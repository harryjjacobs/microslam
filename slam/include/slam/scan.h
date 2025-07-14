/*
 * slam.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef SLAM_SCAN_H_
#define SLAM_SCAN_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "types.h"

void scan_reset(scan_t *scan);

void scan_reset(scan_t *scan);

void scan_add(scan_t *scan, uint16_t bearing, uint16_t range);

bool scan_valid(const scan_t *scan, int index);

#endif /* SLAM_SCAN_H_ */
