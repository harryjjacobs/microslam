#include <log/log.h>
#include <math.h>
#include <microslam/occupancy_quadtree.h>
#include <microslam/utils.h>
#include <stdio.h>

#ifndef OCCUPANCY_QUADTREE_FLOAT_EPSILON
#define OCCUPANCY_QUADTREE_FLOAT_EPSILON 0.0001
#endif

void occupancy_quadtree_init(occupancy_quadtree_t *quadtree, float x, float y,
                             float size, unsigned char max_depth) {
  quadtree->occupancy = OCCUPANCY_FREE;
  quadtree->max_depth = max_depth;
  quadtree->depth = 0;
  quadtree->size = size;
  quadtree->x = x;
  quadtree->y = y;
  quadtree->log_odds = 0.0f;
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
                                                float x, float y,
                                                float log_odds) {
  // outside the bounds of the quadtree
  if (x < quadtree->x - quadtree->size / 2.0f ||
      x > quadtree->x + quadtree->size / 2.0f ||
      y < quadtree->y - quadtree->size / 2.0f ||
      y > quadtree->y + quadtree->size / 2.0f) {
    log_error(
        "(%f, %f) out of bounds of quadtree. quadtree of size %f at (%f, %f)",
        x, y, quadtree->size, quadtree->x, quadtree->y);
    return NULL;
  }

  // this is a leaf node, stop dividing and update the log odds
  if (quadtree->depth >= quadtree->max_depth) {
    log_debug("updating leaf node log odds to %f at (%f, %f)", log_odds, x, y);
    quadtree->log_odds += log_odds;  // TODO: clamp log odds?
    // update the occupancy
    if (quadtree->log_odds > 0.0f) {
      quadtree->occupancy = OCCUPANCY_OCCUPIED;
    } else {
      quadtree->occupancy = OCCUPANCY_FREE;
    }
    return quadtree;
  }

  // order of children is left to right, top to bottom
  unsigned char quadrant_x = (x >= quadtree->x);
  unsigned char quadrant_y = (y >= quadtree->y);
  unsigned char quad_idx = 2 * quadrant_y + quadrant_x;

  if (quadtree->children[quad_idx] == NULL) {
    quadtree->children[quad_idx] =
        (occupancy_quadtree_t *)malloc(sizeof(occupancy_quadtree_t));
    if (quadtree->children[quad_idx] == NULL) {
      fprintf(stderr, "failed to malloc quadtree\n");
      exit(EXIT_FAILURE);
    }

    float halfsize = quadtree->size / 2.0f;
    float quartersize = halfsize / 2.0f;
    occupancy_quadtree_init(quadtree->children[quad_idx],
                            quadtree->x - quartersize + quadrant_x * halfsize,
                            quadtree->y - quartersize + quadrant_y * halfsize,
                            halfsize, quadtree->max_depth);
    quadtree->children[quad_idx]->depth = quadtree->depth + 1;
  }

  occupancy_quadtree_t *leaf =
      occupancy_quadtree_update(quadtree->children[quad_idx], x, y, log_odds);
  if (leaf == NULL) {
    fprintf(stderr, "unexpected NULL occurred when updating quadtree\n");
  }

  if (quadtree->children[quad_idx]->occupancy == OCCUPANCY_FREE) {
    log_debug("removing free leaf node at (%f, %f)",
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
                                              float x, float y) {
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

  unsigned char quadrant_x = (x >= quadtree->x);
  unsigned char quadrant_y = (y >= quadtree->y);
  unsigned char quad_idx = 2 * quadrant_y + quadrant_x;

  if (quadtree->children[quad_idx] == NULL) {
    return NULL;
  }

  return occupancy_quadtree_find(quadtree->children[quad_idx], x, y);
}

float occupancy_quadtree_node_min_dist_squared(occupancy_quadtree_t *quadtree,
                                               float x, float y) {
  float half_size = quadtree->size / 2.0f;
  float min_x = quadtree->x - half_size;
  float max_x = quadtree->x + half_size;
  float min_y = quadtree->y - half_size;
  float max_y = quadtree->y + half_size;

  float dx = fmaxf(0.0f, fmaxf(min_x - x, x - max_x));
  float dy = fmaxf(0.0f, fmaxf(min_y - y, y - max_y));
  return dx * dx + dy * dy;
}

void occupancy_quadtree_nearest_rec(occupancy_quadtree_t *quadtree, float x,
                                    float y, occupancy_quadtree_t **nearest,
                                    float *distance) {
  if (quadtree == NULL) {
    return;
  }

  // return early if the distance to the box is greater than the current
  float box_dist_sq = occupancy_quadtree_node_min_dist_squared(quadtree, x, y);
  if (box_dist_sq > *distance) return;

  if (quadtree->depth >= quadtree->max_depth) {  // leaf node
    // float dx = fmin(fabsf(x - quadtree->x),
    //                 fmin(fabsf(x - quadtree->x - quadtree->size / 2.0f),
    //                      fabsf(x - quadtree->x + quadtree->size / 2.0f)));
    // float dy = fmin(fabsf(y - quadtree->y),
    //                 fmin(fabsf(y - quadtree->y - quadtree->size / 2.0f),
    //                      fabsf(y - quadtree->y + quadtree->size / 2.0f)));
    float dx = fabsf(x - quadtree->x);
    float dy = fabsf(y - quadtree->y);
    float dist = dx * dx + dy * dy;
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
                                                 float x, float y,
                                                 float *distance) {
  occupancy_quadtree_t *nearest = NULL;
  *distance = INFINITY;
  occupancy_quadtree_nearest_rec(quadtree, x, y, &nearest, distance);
  if (nearest != NULL) {
    *distance = sqrtf(*distance);
  }
  return nearest;
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
                                                 float x, float y, float dx,
                                                 float dy, float max_range,
                                                 float *distance) {
  if (quadtree == NULL) {
    return NULL;
  }

  float t;
  if (!ray_intersects_aabb(quadtree->x - quadtree->size / 2.0f,
                           quadtree->y - quadtree->size / 2.0f,
                           quadtree->x + quadtree->size / 2.0f,
                           quadtree->y + quadtree->size / 2.0f, x, y, dx, dy,
                           &t)) {
    return NULL;
  }

  if (quadtree->depth >= quadtree->max_depth && t < max_range) {
    if (distance != NULL) {
      *distance = t * sqrtf(dx * dx + dy * dy);
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