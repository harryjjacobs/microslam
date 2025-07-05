#ifndef MICROSLAM_SLAM_SYSTEM_H
#define MICROSLAM_SLAM_SYSTEM_H

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

// microslam includes
#include "slam/map.h"
#include "slam/occupancy_quadtree.h"
#include "slam/scan_matching.h"
#include "slam/types.h"
#include "state.h"

#define LED_BLINK_PERIOD 5000

#define MAP_SIZE 4.0f
#define MAP_DEPTH 7
#define MAP_LEAF_SIZE (MAP_SIZE / (1 << MAP_DEPTH))

void slam_system_init(slam_system_t *state);

void slam_system_process(slam_system_t *system, scan_t *scan);

#endif
