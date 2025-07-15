#include "slam/system.h"

#include <math.h>
#include <stdbool.h>

#include "slam/logging.h"
#include "slam/scan.h"
#include "slam/weighted_scan_matching.h"

typedef enum {
  LOCALISATION_INITIALISING = 0,
  LOCALISATION_SUCCESSFUL,
  LOCALISATION_FAILED,
} slam_localisation_result_t;

// declarations of static functions
static void robot_pose_init(robot_pose_t *pose);
static void map_init(occupancy_quadtree_t *map);
static bool should_add_key_pose(slam_system_t *system, pose_t *pose);
static void add_key_pose(slam_system_t *system, robot_pose_t *pose);
static slam_localisation_result_t localise(slam_system_t *system, scan_t *scan);

void slam_system_init(slam_system_t *state) {
  robot_pose_init(&state->pose);
  map_init(&state->map);

  state->key_pose_id = 0;
  state->key_poses = NULL;

  INFO("SLAM system initialized with map size %.2f and depth %d", MAP_SIZE,
       MAP_DEPTH);
}

void slam_system_process(slam_system_t *system, pose_t *odometry,
                         scan_t *scan) {
  system->pose.pose.x += odometry->x;
  system->pose.pose.y += odometry->y;
  system->pose.pose.r += odometry->r;

  if (should_add_key_pose(system, &system->pose.pose)) {
    add_key_pose(system, &system->pose);

    slam_localisation_result_t localisation_result = localise(system, scan);

    const float certainty =
        1.0f / (system->pose.error.x * system->pose.error.x +
                system->pose.error.y * system->pose.error.y +
                system->pose.error.r * system->pose.error.r);
    switch (localisation_result) {
      case LOCALISATION_INITIALISING:
        map_add_scan(&system->map, scan, &system->pose.pose,
                     system->key_pose_id, certainty);
        system->pose.error.x *= 1.1f;  // increase error estimate
        system->pose.error.y *= 1.1f;
        system->pose.error.r *= 1.1f;
        break;
      case LOCALISATION_SUCCESSFUL:
        map_add_scan(&system->map, scan, &system->pose.pose,
                     system->key_pose_id, certainty);
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

static bool should_add_key_pose(slam_system_t *system, pose_t *pose) {
  if (system->key_poses == NULL) {
    return true;  // no key poses yet
  }

  float distance =
      hypotf(pose->x - system->key_poses[system->key_pose_id].pose.x,
             pose->y - system->key_poses[system->key_pose_id].pose.y);
  float angle_diff =
      fabsf(pose->r - system->key_poses[system->key_pose_id].pose.r);

  return (distance >= system->params.key_pose_distance ||
          angle_diff >= system->params.key_pose_angle);
}

static void add_key_pose(slam_system_t *system, robot_pose_t *pose) {
  if (system->key_poses == NULL) {
    system->key_poses_capacity = 16;
    system->key_poses = (robot_pose_t *)malloc(sizeof(robot_pose_t) *
                                               system->key_poses_capacity);
    if (system->key_poses == NULL) {
      FATAL("Failed to allocate memory for key poses");
    }
  } else if (system->key_pose_id >= system->key_poses_capacity) {
    // resize the key poses array
    size_t new_capacity = system->key_poses_capacity * 2;
    robot_pose_t *new_key_poses = (robot_pose_t *)realloc(
        system->key_poses, sizeof(robot_pose_t) * new_capacity);
    if (new_key_poses == NULL) {
      FATAL("Failed to reallocate memory for key poses");
      return;
    }
    system->key_poses = new_key_poses;
    system->key_poses_capacity = new_capacity;
  }

  // add the new key pose
  system->key_poses[system->key_pose_id] = *pose;
  system->key_pose_id++;
  INFO("Added key pose %zu at (%d, %d, %.6f)", system->key_pose_id - 1,
       pose->pose.x, pose->pose.y, pose->pose.r);
}

static slam_localisation_result_t localise(slam_system_t *system,
                                           scan_t *scan) {
  // if not enough data to localise yet, just add the scan to the map
  float entropy = map_entropy(&system->map);
  if (entropy < 0.0005f * MAP_LEAF_SIZE) {
    return LOCALISATION_INITIALISING;
  }

  robot_pose_t pose_estimate;
  if (scan_matching_match(scan, &system->lidar, &system->map,
                          &system->pose.pose, &pose_estimate,
                          system->params.scan_matching_iterations)) {
    system->pose = pose_estimate;
    // TODO: update pose error based on scan matching result
    return LOCALISATION_SUCCESSFUL;
  }

  return LOCALISATION_FAILED;
}
