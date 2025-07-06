#include "slam/serialisation.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "slam/logging.h"

static int buffer_grow(serialisation_buffer_t *buf, size_t min_capacity) {
  if (buf->capacity >= min_capacity) return 0;

  size_t new_capacity = buf->capacity > 0 ? buf->capacity : 64;
  while (new_capacity < min_capacity) {
    new_capacity *= 2;
  }

  char *new_data = realloc(buf->data, new_capacity);
  if (!new_data) {
    FATAL("Failed to realloc buffer to %zu bytes", new_capacity);
    return -1;
  }

  buf->data = new_data;
  buf->capacity = new_capacity;
  return 0;
}

static int buffer_append(serialisation_buffer_t *buf, const char *data,
                         size_t data_size) {
  if (buffer_grow(buf, buf->length + data_size) < 0) {
    return -1;
  }
  memcpy(buf->data + buf->length, data, data_size);
  buf->length += data_size;
  return 0;
}

static int buffer_read(const serialisation_buffer_t *buf, size_t *offset,
                       char *data, size_t data_size) {
  if (*offset + data_size > buf->length) {
    ERROR("serialisation_buffer_t read out of bounds");
    return -1;
  }
  memcpy(data, buf->data + *offset, data_size);
  *offset += data_size;
  return 0;
}

static int append_u8(serialisation_buffer_t *buf, uint8_t value) {
  char data[1] = {(char)value};
  return buffer_append(buf, data, 1);
}

static int append_u16_be(serialisation_buffer_t *buf, uint16_t value) {
  char data[2] = {(char)((value >> 8) & 0xFF), (char)(value & 0xFF)};
  return buffer_append(buf, data, 2);
}

static int append_u32_be(serialisation_buffer_t *buf, uint32_t value) {
  char data[4] = {(char)((value >> 24) & 0xFF), (char)((value >> 16) & 0xFF),
                  (char)((value >> 8) & 0xFF), (char)(value & 0xFF)};
  return buffer_append(buf, data, 4);
}

static int append_float_be(serialisation_buffer_t *buf, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return append_u32_be(buf, bits);
}

static int read_u8(const serialisation_buffer_t *buf, size_t *offset,
                   uint8_t *value) {
  if (*offset + 1 > buf->length) {
    ERROR("serialisation_buffer_t read out of bounds for u8");
    return -1;
  }
  *value = (uint8_t)buf->data[*offset];
  (*offset)++;
  return 0;
}

static int read_u16_be(const serialisation_buffer_t *buf, size_t *offset,
                       uint16_t *value) {
  char data[2];
  if (buffer_read(buf, offset, data, 2) < 0) return -1;
  *value = ((uint8_t)data[0] << 8) | (uint8_t)data[1];
  return 0;
}

static int read_s16_be(const serialisation_buffer_t *buf, size_t *offset,
                       int16_t *value) {
  uint16_t u_value;
  if (read_u16_be(buf, offset, &u_value) < 0) return -1;
  *value = (int16_t)u_value;
  return 0;
}

static int read_u32_be(const serialisation_buffer_t *buf, size_t *offset,
                       uint32_t *value) {
  char data[4];
  if (buffer_read(buf, offset, data, 4) < 0) return -1;
  *value = ((uint8_t)data[0] << 24) | ((uint8_t)data[1] << 16) |
           ((uint8_t)data[2] << 8) | (uint8_t)data[3];
  return 0;
}

// currently unused
// static int read_float_be(const serialisation_buffer_t *buf, size_t *offset,
//                          float *value) {
//   uint32_t bits;
//   if (read_u32_be(buf, offset, &bits) < 0) return -1;
//   memcpy(value, &bits, sizeof(bits));
//   return 0;
// }

void serialisation_buffer_init(serialisation_buffer_t *buf) {
  buf->data = NULL;
  buf->length = 0;
  buf->capacity = 0;
}

void serialisation_buffer_free(serialisation_buffer_t *buf) {
  free(buf->data);
  buf->data = NULL;
  buf->length = 0;
  buf->capacity = 0;
}

int write_header(serialisation_buffer_t *buf, const char id[4]) {
  if (buf == NULL || id == NULL) {
    ERROR("Invalid buffer or ID for writing header");
    return -1;
  }

  // reset buffer length for fresh write
  buf->length = 0;

  // 4 for ID + 2 for length placeholder
  if (buffer_grow(buf, 6) < 0) return -1;

  memcpy(buf->data, id, 4);
  buf->length = 6;  // include length placeholder bytes

  // zero the length bytes
  buf->data[4] = 0;
  buf->data[5] = 0;

  return 0;
}

int write_header_length(serialisation_buffer_t *buf) {
  if (buf == NULL || buf->data == NULL || buf->length < 6) {
    ERROR("Invalid buffer for writing header length");
    return -1;
  }

  if (buf->length > UINT16_MAX) {
    ERROR("serialisation_buffer_t size exceeds maximum length");
    return -1;
  }

  uint16_t length = (uint16_t)(buf->length);
  buf->data[4] = (length >> 8) & 0xFF;
  buf->data[5] = length & 0xFF;

  return 0;
}

int read_header(const serialisation_buffer_t *buf, size_t offset, char id[4],
                uint16_t *length) {
  if (buf == NULL || length == NULL || buf->length < 6) {
    ERROR("Invalid buffer, offset, or length for reading header");
    return -1;
  }

  memcpy(id, buf->data + offset, 4);

  if (read_u16_be(buf, &(size_t){offset + 4}, length) < 0) {
    ERROR("Failed to read length from buffer");
    return -1;
  }

  return 0;
}

static int serialise_quadtree_rec(const occupancy_quadtree_t *tree,
                                  serialisation_buffer_t *buf) {
  if (tree == NULL) {
    return append_u8(buf, 0);
  }

  if (append_u8(buf, 1) < 0) return -1;

  if (append_u8(buf, tree->max_depth) < 0) return -1;
  if (append_float_be(buf, tree->size) < 0) return -1;
  if (append_float_be(buf, tree->x) < 0) return -1;
  if (append_float_be(buf, tree->y) < 0) return -1;
  if (append_u8(buf, tree->depth) < 0) return -1;
  if (append_u32_be(buf, tree->occupancy) < 0) return -1;
  if (append_float_be(buf, tree->log_odds) < 0) return -1;

  for (size_t i = 0; i < 4; i++) {
    if (serialise_quadtree_rec(tree->children[i], buf) < 0) return -1;
  }

  return 0;
}

int serialise_quadtree(const occupancy_quadtree_t *tree,
                       serialisation_buffer_t *buf) {
  if (tree == NULL || buf == NULL) {
    ERROR("Invalid tree or buffer for serialisation");
    return -1;
  }
  return serialise_quadtree_rec(tree, buf);
}

static int deserialise_quadtree_rec(const serialisation_buffer_t *buf,
                                    size_t *offset,
                                    occupancy_quadtree_t **tree) {
  uint8_t is_not_null;
  if (read_u8(buf, offset, &is_not_null) < 0) {
    DEBUG("Failed to read is_null flag for quadtree node");
    return -1;
  }

  if (is_not_null == 0) {
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

  if (read_u8(buf, offset, &(*tree)->max_depth) < 0) return -1;
  if (read_u16_be(buf, offset, &(*tree)->size) < 0) return -1;
  if (read_s16_be(buf, offset, &(*tree)->x) < 0) return -1;
  if (read_s16_be(buf, offset, &(*tree)->y) < 0) return -1;
  if (read_u8(buf, offset, &(*tree)->depth) < 0) return -1;

  uint32_t occupancy;
  if (read_u32_be(buf, offset, &occupancy) < 0) {
    free(*tree);
    return -1;
  }
  (*tree)->occupancy = occupancy;

  if (read_s16_be(buf, offset, &(*tree)->log_odds) < 0) {
    free(*tree);
    return -1;
  }

  for (size_t i = 0; i < 4; i++) {
    (*tree)->children[i] = NULL;
    occupancy_quadtree_t *child = NULL;
    int result = deserialise_quadtree_rec(buf, offset, &child);
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

int deserialise_quadtree(const serialisation_buffer_t *buf, size_t *offset,
                         occupancy_quadtree_t *tree) {
  if (buf == NULL || buf->length == 0 || tree == NULL) {
    ERROR("Invalid buffer or tree for deserialisation");
    return -1;
  }

  int result = deserialise_quadtree_rec(buf, offset, &tree);
  if (result < 0) return result;
  if (*offset != buf->length) {
    ERROR("Deserialisation did not consume the entire buffer");
    return -1;
  }
  return 0;
}

int serialise_scan(const scan_t *scan, serialisation_buffer_t *buf) {
  if (scan == NULL || buf == NULL) {
    ERROR("Invalid scan or buffer for serialisation");
    return -1;
  }

  if (append_u16_be(buf, (uint16_t)scan->hits) < 0) return -1;

  for (int i = 0; i < 360; i++) {
    if (append_float_be(buf, scan->range[i]) < 0) return -1;
  }

  return 0;
}

int deserialise_scan(const serialisation_buffer_t *buf, size_t *offset,
                     scan_t *scan) {
  if (buf == NULL || offset == NULL || scan == NULL) {
    ERROR("Invalid buffer or offset or scan for deserialisation");
    return -1;
  }

  // Check minimum size to read hits + 360 floats:
  size_t needed = sizeof(scan->hits) + 360 * sizeof(float);
  if (*offset + needed > buf->length) {
    ERROR("Buffer too small for scan deserialisation");
    return -1;
  }

  if (read_u16_be(buf, offset, &scan->hits) < 0) return -1;

  for (int i = 0; i < 360; i++) {
    if (read_u16_be(buf, offset, &scan->range[i]) < 0) return -1;
  }

  return 0;
}
