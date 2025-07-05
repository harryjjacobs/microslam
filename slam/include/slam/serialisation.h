// clang-format off
/*
 * serialisation.h
 *
 *  Created on: Jul 3, 2025
 *      Author: harryjjacobs
 *
 * @brief For serialising and deserialising structures used in SLAM.
 *
 * @details There are also functions available for writing and reading headers
 * to ensure the data is in the correct format. A header consists of a
 * 4-byte identifier followed by an unsigned 2 byte integer for the length of
 * the data. The size should include the header itself. All data is stored in
 * big-endian format.
 *
 * @example
 * char *buffer;
 * size_t buffer_pos;
 * write_header(&buffer, &buffer_pos, SLAM_SERIALISATION_ID_QUADTREE);
 * serialise_quadtree(tree, &buffer, &buffer_pos);
 * write_header_length(&buffer, buffer_pos);
 *
 * // To read the data back:
 * size_t length;
 * char id[SLAM_SERIALISATION_ID_LEN];
 * read_header(buffer, buffer_pos, id, &length);
 * if (strncmp(id, SLAM_SERIALISATION_ID_QUADTREE, SLAM_SERIALISATION_ID_LEN) == 0) {
 *   occupancy_quadtree_t tree;
 *   deserialise_quadtree(buffer + SLAM_SERIALISATION_ID_LEN + 2, length, &tree);
 * }
 * 
 */
// clang-format on

#ifndef SLAM_SERIALISATION_H_
#define SLAM_SERIALISATION_H_

#include <stddef.h>

#include "occupancy_quadtree.h"
#include "types.h"

#define SLAM_SERIALISATION_ID_LEN 4
// clang-format off
static const char SLAM_SERIALISATION_ID_QUADTREE[SLAM_SERIALISATION_ID_LEN] = {'Q', 'T', 'R', 'E'};
// clang-format on

/**
 * @brief Writes a header to an empty buffer. This will initialise the length
 * field to 0, which should be updated later using `write_header_length`.
 *
 * The header consists of a 4-byte identifier followed by an unsigned
 * 2-byte integer for the length of the data. All data is stored in big-endian
 * format.
 *
 * @param buffer Pointer to a pointer where the header will be written.
 * @param buffer_len Pointer to a size_t where the size of the buffer will be
 * stored.
 * @param id A 4-byte identifier (does not need to be null-terminated).
 */
int write_header(char **buffer, size_t *buffer_len, const char id[4]);

/**
 * @brief Writes the length of the data to the buffer. This should be called
 * after all data has been written to the buffer, and after the header has
 * been written.
 *
 * @param buffer Pointer to the buffer containing the header and data.
 * @param buffer_len The full size of the buffer, including the header.
 * @param length
 */
int write_header_length(char **buffer, size_t buffer_len);

/**
 * @brief Attempts to read the header bytes from the buffer. The caller should
 * check whether the ID matches the expected ID and whether the length is
 * valid. The buffer must be at least SLAM_SERIALISATION_ID_LEN + 2 bytes.
 *
 * @param buffer Pointer to the buffer containing the header.
 * @param buffer_len Size of the buffer.
 * @param id Pointer to a string where the ID will be stored.
 * @param length Pointer to a size_t where the length of the data will be
 * stored.
 */
int read_header(const char *buffer, size_t buffer_len, const char id[4],
                size_t *length);

/**
 * @brief Serialises the occupancy quadtree to a byte array.
 *
 * @details `buffer_pos` should be initialised to before calling this function.
 * It will be used as the initial size of the buffer, and will be updated to the
 * final size of the buffer after serialisation. If the buffer already
 * has data, the serialised data will be appended to it provided that
 * buffer_pos is set to the length of the existing data.
 *
 * @param tree Pointer to the occupancy quadtree to serialise.
 * @param buffer Pointer to a pointer where the serialised data will be stored.
 * The buffer will be allocated dynamically.
 * @param buffer_pos Pointer to a size_t where the size of the serialised data
 * will be stored. This should be initialised before calling this function.
 * @return Returns 0 on success, or a negative value on failure.
 */
int serialise_quadtree(const occupancy_quadtree_t *tree, char **buffer,
                       size_t *buffer_pos);

/**
 * @brief Deserialises a byte array into an occupancy quadtree.
 *
 * @param buffer Pointer to the byte array containing the serialised quadtree
 * data.
 * @param buffer_len Size of the byte array.
 * @param tree Pointer to the occupancy quadtree where the deserialised data
 * will be stored.
 * @return Returns 0 on success, or a negative value on failure.
 */
int deserialise_quadtree(const char *buffer, size_t buffer_len,
                         occupancy_quadtree_t *tree);

#endif
