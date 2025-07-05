/*
 * serialisation.h
 *
 *  Created on: Jul 3, 2025
 *      Author: harryjjacobs
 */

#ifndef MICROSLAM_SERIALISATION_H_
#define MICROSLAM_SERIALISATION_H_

#include <stddef.h>

#include "occupancy_quadtree.h"
#include "types.h"

/**
 * @brief Serialises the occupancy quadtree to a byte array.
 *
 * @param tree Pointer to the occupancy quadtree to serialise.
 * @param buffer Pointer to a pointer where the serialised data will be stored.
 * The buffer will be allocated dynamically.
 * @param buffer_size Pointer to a size_t where the size of the serialised data
 * will be stored.
 * @return Returns 0 on success, or a negative value on failure.
 */
int serialise_quadtree(const occupancy_quadtree_t *tree, char **buffer,
                       size_t *buffer_size);

/**
 * @brief Deserialises a byte array into an occupancy quadtree.
 *
 * @param buffer Pointer to the byte array containing the serialised quadtree
 * data.
 * @param buffer_size Size of the byte array.
 * @param tree Pointer to the occupancy quadtree where the deserialised data
 * will be stored.
 * @return Returns 0 on success, or a negative value on failure.
 */
int deserialise_quadtree(const char *buffer, size_t buffer_size,
                         occupancy_quadtree_t *tree);

#endif
