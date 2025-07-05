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
 */
// clang-format on

#ifndef SLAM_SERIALISATION_H_
#define SLAM_SERIALISATION_H_

#include <stddef.h>
#include <stdint.h>

#include "occupancy_quadtree.h"
#include "types.h"

#define SLAM_SERIALISATION_ID_LEN 4
#define SLAM_SERIALISATION_HEADER_LEN (SLAM_SERIALISATION_ID_LEN + 2)

// clang-format off
static const char SLAM_SERIALISATION_ID_QUADTREE[SLAM_SERIALISATION_ID_LEN] = {'Q', 'T', 'R', 'E'};
static const char SLAM_SERIALISATION_ID_SCAN[SLAM_SERIALISATION_ID_LEN] = {'S', 'C', 'A', 'N'};
// clang-format on

typedef struct {
  char *data;
  size_t length;    // bytes used
  size_t capacity;  // bytes allocated
} serialisation_buffer_t;

/**
 * @brief Initializes a serialisation buffer. Should be called before using
 * the buffer with any serialisation functions.
 *
 * @param buf
 */
void serialisation_buffer_init(serialisation_buffer_t *buf);

/**
 * @brief Frees the memory allocated for a serialisation buffer.
 *
 * @param buf
 */
void serialisation_buffer_free(serialisation_buffer_t *buf);

/**
 * @brief Writes a fixed-size header to a buffer.
 *
 * The header consists of:
 * - A 4-byte identifier (not null-terminated)
 * - A 2-byte unsigned integer representing the length of the data
 *
 * All multi-byte values are stored in big-endian format.
 *
 * This function initializes the buffer with the header and sets the length
 * field to zero, which should be updated later by calling
 * `write_header_length`.
 *
 * @param buf Pointer to the buffer where the header will be written. It should
 * be empty or newly allocated.
 * @param id A 4-byte identifier (e.g., "SLAM", "DATA").
 * @return 0 on success, negative on failure.
 */
int write_header(serialisation_buffer_t *buf, const char id[4]);

/**
 * @brief Updates the length field in the header to match the current buffer
 * size.
 *
 * This function should be called after all data has been written to the buffer,
 * to finalize the length field in the header.
 *
 * @param buf Pointer to the buffer containing the header and serialized data.
 * @return 0 on success, negative on failure.
 */
int write_header_length(serialisation_buffer_t *buf);

/**
 * @brief Reads the header bytes from a buffer.
 *
 * This function extracts the 4-byte ID and 2-byte length field from the buffer.
 * The caller should check if the ID matches the expected value and whether the
 * length is valid for further processing.
 *
 * The buffer must be at least 6 bytes long (4 bytes ID + 2 bytes length).
 *
 * @param buf Pointer to the buffer containing the header.
 * @param offset The offset in the buffer where the header starts.
 * @param id_out serialisation_buffer_t to store the extracted 4-byte ID.
 * @param length_out Pointer to store the extracted length value.
 * @return 0 on success, negative on failure.
 */
int read_header(const serialisation_buffer_t *buf, size_t offset,
                char id_out[4], uint16_t *length_out);

/**
 * @brief Serializes an occupancy quadtree into a byte buffer.
 *
 * The serialized data is appended to the buffer. The buffer's size and capacity
 * will be adjusted as needed.
 *
 * @param tree Pointer to the occupancy quadtree to serialize.
 * @param buf Pointer to the buffer where serialized data will be appended.
 * @return 0 on success, negative on failure.
 */
int serialise_quadtree(const occupancy_quadtree_t *tree,
                       serialisation_buffer_t *buf);

/**
 * @brief Deserializes occupancy quadtree data from a byte buffer.
 *
 * @param buf Pointer to the buffer containing serialized quadtree data.
 * @param offset Pointer to the current reading offset within the buffer;
 * updated during reading.
 * @param tree Pointer to the occupancy quadtree struct where data will be
 * stored.
 * @return 0 on success, negative on failure., size_t *offset
 */
int deserialise_quadtree(const serialisation_buffer_t *buf, size_t *offset,
                         occupancy_quadtree_t *tree);

/**
 * @brief Serializes a scan structure into a byte buffer.
 *
 * The serialized data is appended to the buffer. The buffer's size and capacity
 * will be adjusted as needed.
 *
 * @param scan Pointer to the scan data to serialize.
 * @param buf Pointer to the buffer where serialized data will be appended.
 * @return 0 on success, negative on failure.
 */
int serialise_scan(const scan_t *scan, serialisation_buffer_t *buf);

/**
 * @brief Deserializes scan data from a byte buffer.
 *
 * @param buf Pointer to the buffer containing serialized scan data.
 * @param offset Pointer to the current reading offset within the buffer;
 * updated during reading.
 * @param scan Pointer to the scan structure where deserialized data will be
 * stored.
 * @return 0 on success, negative on failure.
 */
int deserialise_scan(const serialisation_buffer_t *buf, size_t *offset,
                     scan_t *scan);

#endif
