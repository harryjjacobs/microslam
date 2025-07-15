#include <math.h>
#include <slam/logging.h>
#include <slam/occupancy_quadtree.h>
#include <slam/utils.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef OCCUPANCY_QUADTREE_FLOAT_EPSILON
#define OCCUPANCY_QUADTREE_FLOAT_EPSILON 0.0001
#endif

void occupancy_quadtree_init(occupancy_quadtree_t *quadtree, int16_t x,
                             int16_t y, uint16_t size, uint8_t max_depth) {
  // check if the size is a power of two
  if (size == 0 || (size & (size - 1)) != 0) {
    ERROR("quadtree size %hu must be a power of two", size);
    exit(EXIT_FAILURE);
  }

  quadtree->id = 0;
  quadtree->occupancy = OCCUPANCY_FREE;
  quadtree->max_depth = max_depth;
  quadtree->depth = 0;
  quadtree->size = size;
  quadtree->x = x;
  quadtree->y = y;
  quadtree->log_odds = 0;
  for (int i = 0; i < 4; i++) {
    quadtree->children[i] = NULL;
  }
}

void occupancy_quadtree_clear(occupancy_quadtree_t *quadtree) {
  for (int i = 0; i < 4; i++) {
    if (quadtree->children[i] != NULL) {
      occupancy_quadtree_clear(quadtree->children[i]);
      quadtree->children[i] = NULL;
    }
  }
}

occupancy_quadtree_t *occupancy_quadtree_update(occupancy_quadtree_t *quadtree,
                                                int16_t x, int16_t y,
                                                uint16_t id, int32_t log_odds) {
  // outside the bounds of the quadtree
  if (x < (quadtree->x - (quadtree->size >> 1)) ||
      x > (quadtree->x + (quadtree->size >> 1)) ||
      y < (quadtree->y - (quadtree->size >> 1)) ||
      y > (quadtree->y + (quadtree->size >> 1))) {
    ERROR(
        "(%d, %d) out of bounds of quadtree. quadtree of size %hu at (%d, %d)",
        x, y, quadtree->size, quadtree->x, quadtree->y);
    return NULL;
  }

  // this is a leaf node, stop dividing and update the log odds and ID
  if (quadtree->depth >= quadtree->max_depth) {
    quadtree->log_odds = CLAMP(quadtree->log_odds + log_odds, -1000, 1000);
    quadtree->id = id;
    // update the occupancy
    if (quadtree->log_odds > 0) {
      quadtree->occupancy = OCCUPANCY_OCCUPIED;
    } else {
      quadtree->occupancy = OCCUPANCY_FREE;
    }
    return quadtree;
  }

  // order of children is left to right, top to bottom
  uint8_t quadrant_x = (x >= quadtree->x);
  uint8_t quadrant_y = (y >= quadtree->y);
  uint8_t quad_idx = 2 * quadrant_y + quadrant_x;

  if (quadtree->children[quad_idx] == NULL) {
    quadtree->children[quad_idx] =
        (occupancy_quadtree_t *)malloc(sizeof(occupancy_quadtree_t));
    if (quadtree->children[quad_idx] == NULL) {
      ERROR("failed to malloc quadtree");
      exit(EXIT_FAILURE);
    }

    uint16_t halfsize = quadtree->size >> 1;
    uint16_t quartersize = halfsize >> 1;
    occupancy_quadtree_init(quadtree->children[quad_idx],
                            quadtree->x - quartersize + quadrant_x * halfsize,
                            quadtree->y - quartersize + quadrant_y * halfsize,
                            halfsize, quadtree->max_depth);
    quadtree->children[quad_idx]->depth = quadtree->depth + 1;
  }

  occupancy_quadtree_t *leaf = occupancy_quadtree_update(
      quadtree->children[quad_idx], x, y, id, log_odds);
  if (leaf == NULL) {
    ERROR("unexpected NULL occurred when updating quadtree");
  }

  if (quadtree->children[quad_idx]->occupancy == OCCUPANCY_FREE) {
    TRACE("removing free leaf node at (%f, %f)",
          quadtree->children[quad_idx]->x, quadtree->children[quad_idx]->y);
    // child node is free, free the memory
    free(quadtree->children[quad_idx]);
    quadtree->children[quad_idx] = NULL;
  }

  // update occupancy of this node based on its children
  if ((quadtree->children[0] != NULL &&
       quadtree->children[0]->occupancy == OCCUPANCY_OCCUPIED) &&
      (quadtree->children[1] != NULL &&
       quadtree->children[1]->occupancy == OCCUPANCY_OCCUPIED) &&
      (quadtree->children[2] != NULL &&
       quadtree->children[2]->occupancy == OCCUPANCY_OCCUPIED) &&
      (quadtree->children[3] != NULL &&
       quadtree->children[3]->occupancy == OCCUPANCY_OCCUPIED)) {
    quadtree->occupancy = OCCUPANCY_OCCUPIED;
  } else if ((quadtree->children[0] != NULL &&
              quadtree->children[0]->occupancy > OCCUPANCY_FREE) ||
             (quadtree->children[1] != NULL &&
              quadtree->children[1]->occupancy > OCCUPANCY_FREE) ||
             (quadtree->children[2] != NULL &&
              quadtree->children[2]->occupancy > OCCUPANCY_FREE) ||
             (quadtree->children[3] != NULL &&
              quadtree->children[3]->occupancy > OCCUPANCY_FREE)) {
    quadtree->occupancy = OCCUPANCY_MIXED;
  } else {
    quadtree->occupancy = OCCUPANCY_FREE;
  }

  return leaf;
}

occupancy_quadtree_t *occupancy_quadtree_find(occupancy_quadtree_t *quadtree,
                                              int16_t x, int16_t y) {
  // check if the point is within the bounds of the quadtree
  if (x < quadtree->x - quadtree->size / 2.0f ||
      x > quadtree->x + quadtree->size / 2.0f ||
      y < quadtree->y - quadtree->size / 2.0f ||
      y > quadtree->y + quadtree->size / 2.0f) {
    return NULL;
  }

  // if this is a leaf node, check the occupancy
  if (quadtree->children[0] == NULL && quadtree->children[1] == NULL &&
      quadtree->children[2] == NULL && quadtree->children[3] == NULL) {
    if (quadtree->occupancy == OCCUPANCY_OCCUPIED) {
      return quadtree;
    }

    if (quadtree->occupancy == OCCUPANCY_FREE) {
      return NULL;
    }
  }

  uint8_t quadrant_x = (x >= quadtree->x);
  uint8_t quadrant_y = (y >= quadtree->y);
  uint8_t quad_idx = 2 * quadrant_y + quadrant_x;

  if (quadtree->children[quad_idx] == NULL) {
    return NULL;
  }

  return occupancy_quadtree_find(quadtree->children[quad_idx], x, y);
}

uint32_t occupancy_quadtree_node_min_dist_squared(
    occupancy_quadtree_t *quadtree, int16_t x, int16_t y) {
  int16_t half_size = quadtree->size >> 1;
  int16_t min_x = quadtree->x - half_size;
  int16_t max_x = quadtree->x + half_size;
  int16_t min_y = quadtree->y - half_size;
  int16_t max_y = quadtree->y + half_size;

  uint32_t dx = MAX(0, MAX(min_x - x, x - max_x));
  uint32_t dy = MAX(0, MAX(min_y - y, y - max_y));
  return (uint32_t)(dx * dx + dy * dy);
}

void occupancy_quadtree_nearest_rec(occupancy_quadtree_t *quadtree, int16_t x,
                                    int16_t y, occupancy_quadtree_t **nearest,
                                    uint32_t *distance) {
  if (quadtree == NULL) {
    return;
  }

  // return early if the distance to the box is greater than the current
  uint32_t box_dist_sq =
      occupancy_quadtree_node_min_dist_squared(quadtree, x, y);
  if (box_dist_sq > *distance) return;

  if (quadtree->depth >= quadtree->max_depth) {  // leaf node
    uint32_t dx = (uint32_t)abs(x - quadtree->x);
    uint32_t dy = (uint32_t)abs(y - quadtree->y);
    uint32_t dist = dx * dx + dy * dy;
    if (*nearest == NULL || dist < *distance) {
      *nearest = quadtree;
      *distance = dist;
    }
    return;
  }

  for (size_t i = 0; i < 4; i++) {
    occupancy_quadtree_nearest_rec(quadtree->children[i], x, y, nearest,
                                   distance);
  }
}

occupancy_quadtree_t *occupancy_quadtree_nearest(occupancy_quadtree_t *quadtree,
                                                 int16_t x, int16_t y,
                                                 uint16_t *distance) {
  occupancy_quadtree_t *nearest = NULL;
  uint32_t distance_sqrd = UINT32_MAX;
  occupancy_quadtree_nearest_rec(quadtree, x, y, &nearest, &distance_sqrd);
  if (nearest != NULL) {
    *distance = (uint16_t)sqrt(distance_sqrd);
  }
  return nearest;
}

typedef struct {
  occupancy_quadtree_t *node;
  uint32_t distance_squared;
} knn_entry_t;

typedef struct {
  knn_entry_t *entries;
  uint16_t count;
  uint16_t k;
} knn_list_t;

void knn_list_try_insert(knn_list_t *knn, occupancy_quadtree_t *node,
                         uint32_t dist_sq) {
  if (knn->count < knn->k) {
    knn->entries[knn->count++] = (knn_entry_t){node, dist_sq};
  } else {
    // find current furthest (max distance)
    uint16_t max_idx = 0;
    for (uint16_t i = 1; i < knn->count; i++) {
      if (knn->entries[i].distance_squared >
          knn->entries[max_idx].distance_squared) {
        max_idx = i;
      }
    }
    if (dist_sq < knn->entries[max_idx].distance_squared) {
      knn->entries[max_idx] = (knn_entry_t){node, dist_sq};
    }
  }
}

uint32_t knn_list_max_distance(const knn_list_t *knn) {
  if (knn->count < knn->k) return UINT32_MAX;
  uint32_t max_dist = 0;
  for (uint16_t i = 0; i < knn->count; i++) {
    if (knn->entries[i].distance_squared > max_dist) {
      max_dist = knn->entries[i].distance_squared;
    }
  }
  return max_dist;
}

void occupancy_quadtree_knn_rec(occupancy_quadtree_t *quadtree, int16_t x,
                                int16_t y, knn_list_t *knn) {
  if (quadtree == NULL) return;

  uint32_t max_allowed_dist = knn_list_max_distance(knn);

  uint32_t box_dist_sq =
      occupancy_quadtree_node_min_dist_squared(quadtree, x, y);
  if (box_dist_sq > max_allowed_dist) return;

  if (quadtree->depth >= quadtree->max_depth) {  // leaf node
    uint32_t dx = (uint32_t)abs(x - quadtree->x);
    uint32_t dy = (uint32_t)abs(y - quadtree->y);
    uint32_t dist = dx * dx + dy * dy;
    knn_list_try_insert(knn, quadtree, dist);
    return;
  }

  for (size_t i = 0; i < 4; i++) {
    occupancy_quadtree_knn_rec(quadtree->children[i], x, y, knn);
  }
}

uint16_t occupancy_quadtree_k_nearest(occupancy_quadtree_t *quadtree, int16_t x,
                                      int16_t y, occupancy_quadtree_t **nearest,
                                      uint16_t k) {
  if (k == 0) {
    ERROR("Invalid parameters for occupancy_quadtree_k_nearest");
    return 0;
  }

  knn_list_t knn;
  knn.entries = (knn_entry_t *)malloc(k * sizeof(knn_entry_t));
  if (knn.entries == NULL) {
    FATAL("Failed to allocate memory for knn entries");
  }

  knn.count = 0;
  knn.k = k;
  occupancy_quadtree_knn_rec(quadtree, x, y, &knn);
  uint16_t found = knn.count < k ? knn.count : k;
  for (uint16_t i = 0; i < found; i++) {
    nearest[i] = knn.entries[i].node;
  }
  free(knn.entries);

  return found;
}

void occupancy_quadtree_iterate_leafs_depth_first(
    occupancy_quadtree_t *quadtree, void *data,
    void (*callback)(occupancy_quadtree_t *quadtree, void *data)) {
  if (quadtree->depth >= quadtree->max_depth) {
    callback(quadtree, data);
    return;
  }

  for (size_t i = 0; i < 4; i++) {
    if (quadtree->children[i] != NULL) {
      occupancy_quadtree_iterate_leafs_depth_first(quadtree->children[i], data,
                                                   callback);
    }
  }
}

occupancy_quadtree_t *occupancy_quadtree_raycast(occupancy_quadtree_t *quadtree,
                                                 int16_t x, int16_t y,
                                                 int16_t dx, int16_t dy,
                                                 uint16_t max_range,
                                                 uint16_t *distance) {
  if (quadtree == NULL) {
    return NULL;
  }

  uint16_t t;
  if (!ray_intersects_aabb(quadtree->x - (quadtree->size >> 1),
                           quadtree->y - (quadtree->size >> 1),
                           quadtree->x + (quadtree->size >> 1),
                           quadtree->y + (quadtree->size >> 1), x, y, dx, dy,
                           &t)) {
    return NULL;
  }

  if (quadtree->depth >= quadtree->max_depth && t < max_range) {
    if (distance != NULL) {
      *distance = (uint16_t)(t * sqrtf(dx * dx + dy * dy));
    }
    return quadtree;
  }

  occupancy_quadtree_t *hit = NULL;
  for (size_t i = 0; i < 4; i++) {
    occupancy_quadtree_t *child = occupancy_quadtree_raycast(
        quadtree->children[i], x, y, dx, dy, max_range, distance);
    if (child != NULL) {
      hit = child;
    }
  }

  return hit;
}
