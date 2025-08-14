#include <math.h>
#include <slam/map.h>
#include <slam/scan.h>
#include <slam/utils.h>
#include <stdio.h>
#include <stdlib.h>

void map_add_scan(occupancy_quadtree_t *occupancy, scan_t *scan, pose_t *pose,
                  uint16_t id, int32_t weight) {
  // step is the leaf size divided by 4 (just needs to be small enough
  // to not miss any cells)
  const uint16_t step = MAX(1, occupancy->size >> (occupancy->max_depth + 1));
  float r, dx, dy;
  for (uint16_t i = 0; i < 360; i++) {
    if (!scan_valid(scan, i)) {
      continue;
    }

    r = pose->r + DEG2RAD(i);

    // update along the ray from the robot to the endpoint of the scan as free
    for (float d = 0; d < scan->range[i]; d += step) {
      dx = d * cos(r);
      dy = d * sin(r);
      occupancy_quadtree_update(occupancy, pose->x + (int16_t)dx,
                                pose->y + (int16_t)dy, id, LOG_ODDS_FREE);
    }
  }

  // update the endpoint of the scan as occupied.
  // we do this as a separate step to avoid overwriting the occupied cells
  // with free cells from nearby rays that hit further away
  for (uint16_t i = 0; i < 360; i++) {
    if (!scan_valid(scan, i)) {
      continue;
    }

    r = pose->r + DEG2RAD(i);
    dx = scan->range[i] * cos(r);
    dy = scan->range[i] * sin(r);

    // update the endpoint of the scan as occupied
    occupancy_quadtree_update(occupancy, pose->x + dx, pose->y + dy, id,
                              LOG_ODDS_OCCUPIED * weight);
  }
}

void entropy_count(occupancy_quadtree_t *leaf, void *count) {
  (void)leaf;
  (*(size_t *)count)++;
}

float map_entropy(occupancy_quadtree_t *occupancy) {
  size_t occupied = 0;
  size_t free = 1 << (2 * occupancy->max_depth);
  occupancy_quadtree_iterate_leafs_depth_first(occupancy, &occupied,
                                               entropy_count);
  float epsilon = 1e-10; // small value to prevent log(0)
  float P_occ = (float)occupied / (occupied + free) + epsilon;
  float P_free = (float)free / (occupied + free) + epsilon;
  return -(P_occ * log(P_occ) + P_free * log(P_free));
}
