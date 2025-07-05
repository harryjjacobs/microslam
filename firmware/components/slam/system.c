#include "system.h"

#include "logging.h"

typedef enum {
  LOCALISATION_INITIALISING = 0,
  LOCALISATION_SUCCESSFUL,
  LOCALISATION_FAILED,
} slam_localisation_result_t;

// declarations of static functions
static void robot_pose_init(robot_pose_t *pose);
static void map_init(occupancy_quadtree_t *map);
static slam_localisation_result_t localise(occupancy_quadtree_t *map,
                                           scan_t *scan, pose_t *current_pose,
                                           pose_t *pose_estimate);

void slam_system_init(slam_system_t *state) {
  robot_pose_init(&state->pose);
  map_init(&state->map);
  scan_reset(&state->scan);

  INFO("SLAM system initialized with map size %.2f and depth %d", MAP_SIZE,
       MAP_DEPTH);
}

void slam_system_process(slam_system_t *system, scan_t *scan) {
  slam_localisation_result_t localisation_result =
      localise(&system->map, scan, &system->pose.pose, &system->pose.pose);

  const float certainty = 1.0f / (system->pose.error.x * system->pose.error.x +
                                  system->pose.error.y * system->pose.error.y +
                                  system->pose.error.r * system->pose.error.r);
  switch (localisation_result) {
    case LOCALISATION_INITIALISING:
      map_add_scan(&system->map, scan, &system->pose.pose, certainty);
      system->pose.error.x *= 1.1f;  // increase error estimate
      system->pose.error.y *= 1.1f;
      system->pose.error.r *= 1.1f;
      break;
    case LOCALISATION_SUCCESSFUL:
      map_add_scan(&system->map, scan, &system->pose.pose, certainty);
      system->pose.error.x *= 0.5f;  // decrease error estimate
      system->pose.error.y *= 0.5f;
      system->pose.error.r *= 0.5f;
      break;
    case LOCALISATION_FAILED:
      ERROR("Localisation failed, increasing error estimate");
      system->pose.error.x *= 1.5f;  // increase error estimate
      system->pose.error.y *= 1.5f;
      system->pose.error.r *= 1.5f;
      break;
  }
}

static void map_init(occupancy_quadtree_t *map) {
  occupancy_quadtree_init(map, 0.0f, 0.0f, MAP_SIZE, MAP_DEPTH);
}

static void robot_pose_init(robot_pose_t *pose) {
  pose->pose.x = 0.0f;
  pose->pose.y = 0.0f;
  pose->pose.r = 0.0f;

  pose->error.x = 0.0f;
  pose->error.y = 0.0f;
  pose->error.r = 0.0f;
}

static slam_localisation_result_t localise(occupancy_quadtree_t *map,
                                           scan_t *scan, pose_t *current_pose,
                                           pose_t *pose_estimate) {
  // if not enough data to localise yet, just add the scan to the map
  float entropy = map_entropy(map);
  if (entropy < 0.05f * MAP_LEAF_SIZE) {
    return LOCALISATION_INITIALISING;
  }

  float score;
  if (scan_matching_match(map, scan, current_pose, pose_estimate, &score,
                          250)) {
    *current_pose = *pose_estimate;
    return LOCALISATION_SUCCESSFUL;
  }

  return LOCALISATION_FAILED;
}