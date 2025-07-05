#include "slam/serialisation.h"

#include <string.h>

#include "slam/logging.h"

void buffer_alloc(char **buffer, size_t *buffer_size, size_t new_size) {
  if (*buffer == NULL || *buffer_size == 0) {
    *buffer = malloc(new_size);
    if (*buffer == NULL) {
      FATAL("Failed to allocate initial buffer");
      return;
    }
    *buffer_size = new_size;
  } else if (new_size > *buffer_size) {
    *buffer = realloc(*buffer, new_size);
    if (*buffer == NULL) {
      FATAL("Failed to grow buffer");
      return;
    }
    *buffer_size = new_size;
  }
}

void buffer_write(char **buffer, size_t *buffer_size, const char *data,
                  size_t data_size) {
  size_t old_size = *buffer_size;
  size_t new_size = *buffer_size + data_size;
  buffer_alloc(buffer, buffer_size, new_size);
  memcpy(*buffer + old_size, data, data_size);
  *buffer_size = new_size;
}

int buffer_read(const char *buffer, size_t buffer_size, size_t *offset,
                char *data, size_t data_size) {
  if (*offset + data_size > buffer_size) {
    ERROR("Buffer read out of bounds");
    return -1;
  }
  memcpy(data, buffer + *offset, data_size);
  *offset += data_size;
  return 0;  // success
}

static int serialise_quadtree_rec(const occupancy_quadtree_t *tree,
                                  char **buffer, size_t *buffer_size) {
  if (tree == NULL) {
    buffer_write(buffer, buffer_size, &(char){0}, sizeof(char));
    return 0;
  }

  buffer_write(buffer, buffer_size, &(char){1}, sizeof(char));

  // write the node's properties

  // TODO: some of these could be written once for the root node and then the
  // rest determined automatically when deserialising; we could save space by
  // not writing them
  buffer_write(buffer, buffer_size, (const char *)&tree->max_depth,
               sizeof(tree->max_depth));
  buffer_write(buffer, buffer_size, (const char *)&tree->size,
               sizeof tree->size);
  buffer_write(buffer, buffer_size, (const char *)&tree->x, sizeof(tree->x));
  buffer_write(buffer, buffer_size, (const char *)&tree->y, sizeof(tree->y));
  buffer_write(buffer, buffer_size, (const char *)&tree->depth,
               sizeof(tree->depth));
  buffer_write(buffer, buffer_size, (const char *)&tree->occupancy,
               sizeof(tree->occupancy));
  buffer_write(buffer, buffer_size, (const char *)&tree->log_odds,
               sizeof(tree->log_odds));

  for (size_t i = 0; i < 4; i++) {
    occupancy_quadtree_t *child = tree->children[i];
    int result = serialise_quadtree_rec(child, buffer, buffer_size);
    if (result < 0) {
      return result;
    }
  }

  return 0;
}

int serialise_quadtree(const occupancy_quadtree_t *tree, char **buffer,
                       size_t *buffer_size) {
  if (tree == NULL || buffer == NULL || buffer_size == 0) {
    ERROR("Invalid tree or buffer for serialisation");
    return -1;
  }

  // serialise the quadtree recursively
  *buffer_size = 0;
  return serialise_quadtree_rec(tree, buffer, buffer_size);
}

static int deserialise_quadtree_rec(const char *buffer, size_t buffer_size,
                                    size_t *offset,
                                    occupancy_quadtree_t **tree) {
  char is_null;
  if (buffer_read(buffer, buffer_size, offset, &is_null, sizeof(is_null)) < 0) {
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

  // read the node's properties
  if (buffer_read(buffer, buffer_size, offset, (char *)&(*tree)->max_depth,
                  sizeof((*tree)->max_depth)) < 0) {
    return -1;
  }
  if (buffer_read(buffer, buffer_size, offset, (char *)&(*tree)->size,
                  sizeof((*tree)->size)) < 0) {
    return -1;
  }
  if (buffer_read(buffer, buffer_size, offset, (char *)&(*tree)->x,
                  sizeof((*tree)->x)) < 0) {
    return -1;
  }
  if (buffer_read(buffer, buffer_size, offset, (char *)&(*tree)->y,
                  sizeof((*tree)->y)) < 0) {
    return -1;
  }
  if (buffer_read(buffer, buffer_size, offset, (char *)&(*tree)->depth,
                  sizeof((*tree)->depth)) < 0) {
    return -1;
  }
  if (buffer_read(buffer, buffer_size, offset, (char *)&(*tree)->occupancy,
                  sizeof((*tree)->occupancy)) < 0) {
    free(*tree);
    return -1;
  }
  if (buffer_read(buffer, buffer_size, offset, (char *)&(*tree)->log_odds,
                  sizeof((*tree)->log_odds)) < 0) {
    free(*tree);
    return -1;
  }

  for (size_t i = 0; i < 4; i++) {
    (*tree)->children[i] = NULL;  // initialise to NULL
    occupancy_quadtree_t *child = NULL;
    int result = deserialise_quadtree_rec(buffer, buffer_size, offset, &child);
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

int deserialise_quadtree(const char *buffer, size_t buffer_size,
                         occupancy_quadtree_t *tree) {
  if (buffer == NULL || buffer_size == 0 || tree == NULL) {
    ERROR("Invalid buffer or tree for deserialisation");
    return -1;
  }

  size_t offset = 0;

  // deserialise the quadtree recursively
  int result = deserialise_quadtree_rec(buffer, buffer_size, &offset, &tree);
  if (result < 0) {
    return result;
  }

  if (offset != buffer_size) {
    ERROR("Deserialisation did not consume the entire buffer");
    return -1;  // not all data was consumed
  }
  return 0;  // success
}
