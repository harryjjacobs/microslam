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

typedef struct scan_t {
  uint16_t range[360];
  uint16_t hits; // the number of valid (non-zero) hits in the scan
} scan_t;

void scan_reset(scan_t *scan);

void scan_reset(scan_t *scan);

void scan_add(scan_t *scan, uint16_t bearing, uint16_t range);

bool scan_valid(const scan_t *scan, int index);

#endif /* SLAM_SCAN_H_ */
