#ifndef SLAM_SYSTEM_H
#define SLAM_SYSTEM_H

#include <stdio.h>

#include "map.h"
#include "occupancy_quadtree.h"
#include "types.h"

#define MAP_SIZE 4096
#define MAP_DEPTH 8
#define MAP_LEAF_SIZE (MAP_SIZE >> MAP_DEPTH)

void slam_system_init(slam_system_t *state);

void slam_system_process(slam_system_t *system, pose_t *odometry, scan_t *scan);

#endif
