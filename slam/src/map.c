#include <math.h>
#include <slam/map.h>
#include <slam/utils.h>
#include <stdio.h>
#include <stdlib.h>

void map_add_scan(occupancy_quadtree_t *occupancy, scan_t *scan, pose_t *pose,
                  float weight) {
  const float step = occupancy->size / (1 << occupancy->max_depth) * 0.1f;
  float r, dx, dy;
  for (unsigned short i = 0; i < 360; i++) {
    if (scan->range[i] < 1e-6) {
      continue;
    }

    r = pose->r + DEG2RAD(i);

    // update along the ray from the robot to the endpoint of the scan as free
    for (float d = 0; d < scan->range[i]; d += step) {
      dx = d * cos(r);
      dy = d * sin(r);
      occupancy_quadtree_update(occupancy, pose->x + dx, pose->y + dy,
                                LOG_ODDS_FREE);
    }
  }

  // update the endpoint of the scan as occupied.
  // we do this as a separate step to avoid overwriting the occupied cells
  // with free cells from nearby rays that hit further away
  for (unsigned short i = 0; i < 360; i++) {
    if (scan->range[i] < 1e-6) {
      continue;
    }

    r = pose->r + DEG2RAD(i);

    dx = scan->range[i] * cos(r);
    dy = scan->range[i] * sin(r);

    // update the endpoint of the scan as occupied
    occupancy_quadtree_update(occupancy, pose->x + dx, pose->y + dy,
                              LOG_ODDS_OCCUPIED * weight);
  }
}

void entropy_count(occupancy_quadtree_t *leaf, void *count) {
  (void)leaf;
  (*(size_t *)count)++;
}

float map_entropy(occupancy_quadtree_t *occupancy) {
  size_t occupied = 0;
  size_t free = pow(2, 2 * occupancy->max_depth);
  occupancy_quadtree_iterate_leafs_depth_first(occupancy, &occupied,
                                               entropy_count);
  float epsilon = 1e-10;  // small value to prevent log(0)
  float P_occ = (float)occupied / (occupied + free) + epsilon;
  float P_free = (float)free / (occupied + free) + epsilon;
  return -(P_occ * log(P_occ) + P_free * log(P_free));
}
