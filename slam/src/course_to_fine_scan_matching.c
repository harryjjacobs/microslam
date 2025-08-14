#include "slam/course_to_fine_scan_matching.h"

#include <math.h>

#include "slam/logging.h"
#include "slam/utils.h"

float evaluate_scan_match(const scan_t *scan, occupancy_quadtree_t *map,
                          uint16_t min_id, pose_t pose, uint8_t depth);

bool course_to_fine_scan_matching_match(const scan_t *scan,
                                        occupancy_quadtree_t *map,
                                        uint16_t max_id,
                                        const robot_pose_t *initial_guess,
                                        robot_pose_t *pose_estimate) {
  const uint16_t map_min_resolution = map->size >> map->max_depth;
  const float min_angle_step = DEG2RAD(0.5f);

  uint16_t search_window_x = MAX(initial_guess->error.x, map_min_resolution);
  uint16_t search_window_y = MAX(initial_guess->error.y, map_min_resolution);
  float search_window_r = MAX(initial_guess->error.r, min_angle_step);

  const int steps = 5; // odd, always symmetric

  pose_t best_pose = initial_guess->pose;

  // how many times we can half the maximum resolution before we reach the
  // minimum resolution
  uint8_t max_level = (uint8_t)fmax(
      ceil(log2f(fmax(search_window_x, search_window_y) / map_min_resolution)),
      ceil(log2f(search_window_r / min_angle_step)));

  for (uint8_t level = 0; level <= max_level; level++) {
    uint16_t step_x = search_window_x / (steps / 2);
    uint16_t step_y = search_window_y / (steps / 2);
    float step_r = search_window_r / (steps / 2);

    float best_score = -INFINITY;
    pose_t current_best_pose = best_pose;

    for (int ix = -(steps / 2); ix <= (steps / 2); ix++) {
      for (int iy = -(steps / 2); iy <= (steps / 2); iy++) {
        for (int itheta = -(steps / 2); itheta <= (steps / 2); itheta++) {
          pose_t candidate = {
              .x = best_pose.x + ix * step_x,
              .y = best_pose.y + iy * step_y,
              .r = clamp_rotation(best_pose.r + itheta * step_r),
          };

          float score =
              evaluate_scan_match(scan, map, max_id, candidate, (uint8_t)level);

          if (score > best_score) {
            best_score = score;
            current_best_pose = candidate;
          }
        }
      }
    }

    best_pose = current_best_pose;

    DEBUG("Course-to-fine scan matching: Level %d: Best score %.2f at pose "
          "(%d,%d, %.9f)",
          level, best_score, best_pose.x, best_pose.y, RAD2DEG(best_pose.r));

    // Halve search window at each refinement level
    search_window_x >>= 1;
    search_window_y >>= 1;
    search_window_r *= 0.5f;
  }

  pose_estimate->pose = best_pose;
  pose_estimate->error.x = search_window_x;
  pose_estimate->error.y = search_window_y;
  pose_estimate->error.r = search_window_r;

  return true;
}

float evaluate_scan_match(const scan_t *scan, occupancy_quadtree_t *map,
                          uint16_t max_id, pose_t pose, uint8_t depth) {
  const uint16_t max_distance = map->size >> depth;

  float score = 0.0f;

  for (int i = 0; i < 360; i++) {
    if (!scan_valid(scan, i))
      continue;

    float angle = pose.r + DEG2RAD(i);
    float range = scan->range[i];
    float x = pose.x + range * cosf(angle);
    float y = pose.y + range * sinf(angle);

    uint16_t distance;
    occupancy_quadtree_t *node =
        occupancy_quadtree_nearest(map, x, y, &distance);
    if (node != NULL && node->id <= max_id && distance < max_distance) {
      score += 1.0f - (float)distance / max_distance;
    }
  }

  return score;
}