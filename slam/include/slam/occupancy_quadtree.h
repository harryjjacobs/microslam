/*
 * occupancy_quadtree.h
 *
 *  Created on: Mar 30, 2025
 *      Author: harryjjacobs
 */

#ifndef SLAM_OCCUPANCY_QUADTREE_H_
#define SLAM_OCCUPANCY_QUADTREE_H_

#include "types.h"

/**
 * @brief Initialize an occupancy quadtree with the given parameters.
 *
 * @details The leaf size can be calculated as size / 2^max_depth.
 *
 * @param quadtree
 * @param x
 * @param y
 * @param size
 * @param max_depth
 */
void occupancy_quadtree_init(occupancy_quadtree_t *quadtree, float x, float y,
                             float size, unsigned char max_depth);

/**
 * @brief Free any memory allocated for the quadtree. This function will
 * recursively free all children of the quadtree. The quadtree itself is not
 * freed. This is the responsibility of the caller.
 *
 * @param quadtree
 */
void occupancy_quadtree_clear(occupancy_quadtree_t *quadtree);

occupancy_quadtree_t *occupancy_quadtree_update(occupancy_quadtree_t *quadtree,
                                                float x, float y,
                                                float log_odds);

occupancy_quadtree_t *occupancy_quadtree_find(occupancy_quadtree_t *quadtree,
                                              float x, float y);

occupancy_quadtree_t *occupancy_quadtree_nearest(occupancy_quadtree_t *quadtree,
                                                 float x, float y,
                                                 float *distance);

void occupancy_quadtree_iterate_leafs_depth_first(
    occupancy_quadtree_t *quadtree, void *data,
    void (*callback)(occupancy_quadtree_t *quadtree, void *data));

occupancy_quadtree_t *occupancy_quadtree_raycast(occupancy_quadtree_t *quadtree,
                                                 float x, float y, float dx,
                                                 float dy, float max_range,
                                                 float *distance);

// void occupancy_quadtree_k_nearest

#endif /* SLAM_OCCUPANCY_QUADTREE_H_ */