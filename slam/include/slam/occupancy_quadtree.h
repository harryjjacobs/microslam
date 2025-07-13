/*
 * occupancy_quadtree.h
 *
 *  Created on: Mar 30, 2025
 *      Author: harryjjacobs
 */

#ifndef SLAM_OCCUPANCY_QUADTREE_H_
#define SLAM_OCCUPANCY_QUADTREE_H_

#include <stdint.h>

#include "types.h"

/**
 * @brief Initialize an occupancy quadtree with the given parameters. The
 * quadtree is initialized with a center point (x, y), a size, and a maximum
 * depth.
 *
 * The size must be a power of two, and be evenly divisible by 2^max_depth.
 *
 * @details The leaf size can be calculated as size / 2^max_depth.
 *
 * @param quadtree The quadtree to initialize.
 * @param x The x coordinate of the center of the quadtree.
 * @param y The y coordinate of the center of the quadtree.
 * @param size The size of the quadtree, which is the length of one side of the
 * quadtree square. Must be a power of two and evenly divisible by 2^max_depth.
 * @param max_depth The maximum depth of the quadtree. This is the number of
 * times the quadtree will be divided into four quadrants. The leaf size can
 * be calculated as: size / (1 << max_depth).
 */
void occupancy_quadtree_init(occupancy_quadtree_t *quadtree, int16_t x,
                             int16_t y, uint16_t size, uint8_t max_depth);

/**
 * @brief Free any memory allocated for the quadtree. This function will
 * recursively free all children of the quadtree.
 *
 * @param quadtree
 */
void occupancy_quadtree_clear(occupancy_quadtree_t *quadtree);

/**
 * @brief Update the occupancy quadtree with a new measurement.
 *
 * @param quadtree The quadtree to update.
 * @param x The x coordinate of the measurement.
 * @param y
 * @param id The unique identifier for the measurement. Leaf nodes in the
 * quadtree will store this ID.
 * @param log_odds The log odds value to update the quadtree with. This is a
 * @return occupancy_quadtree_t*
 */
occupancy_quadtree_t *occupancy_quadtree_update(occupancy_quadtree_t *quadtree,
                                                int16_t x, int16_t y,
                                                uint16_t id, int32_t log_odds);

occupancy_quadtree_t *occupancy_quadtree_find(occupancy_quadtree_t *quadtree,
                                              int16_t x, int16_t y);

occupancy_quadtree_t *occupancy_quadtree_nearest(occupancy_quadtree_t *quadtree,
                                                 int16_t x, int16_t y,
                                                 uint16_t *distance);

/**
 * @brief Iterate over all leaf nodes in the quadtree.
 *
 * @param quadtree The quadtree to iterate over
 * @param data A pointer to any custom data that should be passed to the
 * callback
 * @param callback A function that will be called for each leaf node in the
 * quadtree.
 */
void occupancy_quadtree_iterate_leafs_depth_first(
    occupancy_quadtree_t *quadtree, void *data,
    void (*callback)(occupancy_quadtree_t *quadtree, void *data));

occupancy_quadtree_t *occupancy_quadtree_raycast(occupancy_quadtree_t *quadtree,
                                                 int16_t x, int16_t y,
                                                 int16_t dx, int16_t dy,
                                                 uint16_t max_range,
                                                 uint16_t *distance);

#endif
