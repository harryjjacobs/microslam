#include "slam/serialisation.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "slam/logging.h"

static void buffer_alloc(char **buffer, size_t *buffer_len, size_t new_size) {
  if (*buffer == NULL || *buffer_len == 0) {
    *buffer = malloc(new_size);
    if (*buffer == NULL) {
      FATAL("Failed to allocate initial buffer");
      return;
    }
    *buffer_len = new_size;
  } else if (new_size > *buffer_len) {
    *buffer = realloc(*buffer, new_size);
    if (*buffer == NULL) {
      FATAL("Failed to grow buffer");
      return;
    }
    *buffer_len = new_size;
  }
}

static void buffer_append(char **buffer, size_t *buffer_len, const char *data,
                          size_t data_size) {
  size_t old_size = *buffer_len;
  size_t new_size = *buffer_len + data_size;
  buffer_alloc(buffer, buffer_len, new_size);
  memcpy(*buffer + old_size, data, data_size);
  *buffer_len = new_size;
}

static int buffer_read(const char *buffer, size_t buffer_len, size_t *offset,
                       char *data, size_t data_size) {
  if (*offset + data_size > buffer_len) {
    ERROR("Buffer read out of bounds");
    return -1;
  }
  memcpy(data, buffer + *offset, data_size);
  *offset += data_size;
  return 0;
}

static void append_u8(char **buffer, size_t *buffer_len, uint8_t value) {
  char data[1];
  data[0] = value;
  buffer_append(buffer, buffer_len, data, 1);
}

static void append_u32_be(char **buffer, size_t *buffer_len, uint32_t value) {
  char data[4];
  data[0] = (value >> 24) & 0xFF;
  data[1] = (value >> 16) & 0xFF;
  data[2] = (value >> 8) & 0xFF;
  data[3] = value & 0xFF;
  buffer_append(buffer, buffer_len, data, 4);
}

static void append_float_be(char **buffer, size_t *buffer_len, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  append_u32_be(buffer, buffer_len, bits);
}

static int read_u8(const char *buffer, size_t buffer_len, size_t *offset,
                   uint8_t *value) {
  if (*offset + 1 > buffer_len) {
    ERROR("Buffer read out of bounds for u8");
    return -1;
  }
  *value = (uint8_t)buffer[*offset];
  (*offset)++;
  return 0;
}

static int read_u16_be(const char *buffer, size_t buffer_len, size_t *offset,
                       uint16_t *value) {
  char data[2];
  if (buffer_read(buffer, buffer_len, offset, data, 2) < 0) return -1;
  *value = ((uint8_t)data[0] << 8) | (uint8_t)data[1];
  return 0;
}

static int read_u32_be(const char *buffer, size_t buffer_len, size_t *offset,
                       uint32_t *value) {
  char data[4];
  if (buffer_read(buffer, buffer_len, offset, data, 4) < 0) return -1;
  *value = ((uint8_t)data[0] << 24) | ((uint8_t)data[1] << 16) |
           ((uint8_t)data[2] << 8) | (uint8_t)data[3];
  return 0;
}

static int read_float_be(const char *buffer, size_t buffer_len, size_t *offset,
                         float *value) {
  uint32_t bits;
  if (read_u32_be(buffer, buffer_len, offset, &bits) < 0) return -1;
  memcpy(value, &bits, sizeof(bits));
  return 0;
}

int write_header(char **buffer, size_t *buffer_len, const char id[4]) {
  if (buffer == NULL || id == NULL) {
    ERROR("Invalid buffer or ID for writing header");
    return -1;
  }

  *buffer_len = 0;
  buffer_alloc(buffer, buffer_len, 6);  // 4 bytes for ID + 2 bytes for length
  if (*buffer == NULL) {
    ERROR("Failed to allocate buffer for header");
    return -1;
  }

  // add the id to the buffer in big-endian format
  memcpy(*buffer, id, 4);
  // Initialize the length bytes to zero
  (*buffer)[4] = 0;  // Placeholder for high byte of length
  (*buffer)[5] = 0;  // Placeholder for low byte of length

  return 0;
}

int write_header_length(char **buffer, size_t buffer_len) {
  if (buffer == NULL || *buffer == NULL || buffer_len < 6) {
    ERROR("Invalid buffer for writing header length");
    return -1;
  }

  // the length is stored in the last two bytes of the header

  if (buffer_len > UINT16_MAX) {
    ERROR("Buffer size exceeds maximum length");
    return -1;
  }

  uint16_t length = (uint16_t)(buffer_len);
  (*buffer)[4] = (length >> 8) & 0xFF;  // High byte
  (*buffer)[5] = length & 0xFF;         // Low byte

  return 0;
}

int read_header(const char *buffer, size_t buffer_len, const char id[4],
                size_t *length) {
  if (buffer == NULL || length == NULL || buffer_len < 6) {
    ERROR("Invalid buffer or length for reading header");
    return -1;
  }

  memcpy((char *)id, buffer, 4);

  uint16_t len;
  if (read_u16_be(buffer, buffer_len, &(size_t){4}, &len) < 0) {
    ERROR("Failed to read length from buffer");
    return -1;
  }

  *length = len;
  return 0;
}

static int serialise_quadtree_rec(const occupancy_quadtree_t *tree,
                                  char **buffer, size_t *buffer_pos) {
  if (tree == NULL) {
    buffer_append(buffer, buffer_pos, &(char){0}, sizeof(char));
    return 0;
  }

  buffer_append(buffer, buffer_pos, &(char){1}, sizeof(char));

  append_u8(buffer, buffer_pos, tree->max_depth);
  append_float_be(buffer, buffer_pos, tree->size);
  append_float_be(buffer, buffer_pos, tree->x);
  append_float_be(buffer, buffer_pos, tree->y);
  append_u8(buffer, buffer_pos, tree->depth);
  append_u32_be(buffer, buffer_pos, tree->occupancy);
  append_float_be(buffer, buffer_pos, tree->log_odds);

  for (size_t i = 0; i < 4; i++) {
    occupancy_quadtree_t *child = tree->children[i];
    int result = serialise_quadtree_rec(child, buffer, buffer_pos);
    if (result < 0) {
      return result;
    }
  }

  return 0;
}

int serialise_quadtree(const occupancy_quadtree_t *tree, char **buffer,
                       size_t *buffer_len) {
  if (tree == NULL || buffer == NULL || buffer_len == 0) {
    ERROR("Invalid tree or buffer for serialisation");
    return -1;
  }
  return serialise_quadtree_rec(tree, buffer, buffer_len);
}

static int deserialise_quadtree_rec(const char *buffer, size_t buffer_len,
                                    size_t *offset,
                                    occupancy_quadtree_t **tree) {
  char is_null;
  if (buffer_read(buffer, buffer_len, offset, &is_null, sizeof(is_null)) < 0) {
    DEBUG("Failed to read is_null flag for quadtree node");
    return -1;
  }

  if (is_null == 0) {
    *tree = NULL;
    return 0;
  }

  if (*tree == NULL) {
    *tree = malloc(sizeof(occupancy_quadtree_t));
    if (*tree == NULL) {
      FATAL("Failed to allocate memory for quadtree node");
      return -1;
    }
  }

  if (read_u8(buffer, buffer_len, offset, &(*tree)->max_depth) < 0) return -1;
  if (read_float_be(buffer, buffer_len, offset, &(*tree)->size) < 0) return -1;
  if (read_float_be(buffer, buffer_len, offset, &(*tree)->x) < 0) return -1;
  if (read_float_be(buffer, buffer_len, offset, &(*tree)->y) < 0) return -1;
  if (read_u8(buffer, buffer_len, offset, &(*tree)->depth) < 0) return -1;
  uint32_t occupancy;
  if (read_u32_be(buffer, buffer_len, offset, &occupancy) < 0) {
    free(*tree);
    return -1;
  }
  (*tree)->occupancy = occupancy;
  if (read_float_be(buffer, buffer_len, offset, &(*tree)->log_odds) < 0) {
    free(*tree);
    return -1;
  }

  for (size_t i = 0; i < 4; i++) {
    (*tree)->children[i] = NULL;
    occupancy_quadtree_t *child = NULL;
    int result = deserialise_quadtree_rec(buffer, buffer_len, offset, &child);
    if (result < 0) {
      ERROR("Failed to deserialise child %zu of quadtree", i);
      free(*tree);
      return result;
    }
    if (child != NULL) {
      (*tree)->children[i] = child;
    }
  }

  return 0;
}

int deserialise_quadtree(const char *buffer, size_t buffer_len,
                         occupancy_quadtree_t *tree) {
  if (buffer == NULL || buffer_len == 0 || tree == NULL) {
    ERROR("Invalid buffer or tree for deserialisation");
    return -1;
  }

  size_t offset = 0;
  int result = deserialise_quadtree_rec(buffer, buffer_len, &offset, &tree);
  if (result < 0) return result;
  if (offset != buffer_len) {
    ERROR("Deserialisation did not consume the entire buffer");
    return -1;
  }
  return 0;
}
