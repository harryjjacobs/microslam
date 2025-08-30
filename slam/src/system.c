#include "slam/system.h"

#include <math.h>
#include <stdbool.h>

#include <slam/course_to_fine_scan_matching.h>
#include <slam/logging.h>
#include <slam/scan.h>
#include <slam/weighted_scan_matching.h>

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
static bool should_relocalise(slam_system_params_t *params,
                              robot_pose_t *current_pose,
                              robot_pose_t *estimated_pose);
static slam_localisation_result_t localise(slam_system_t *system, scan_t *scan,
                                           robot_pose_t *estimate);
static slam_localisation_result_t
relocalise(slam_system_t *system, scan_t *scan, robot_pose_t *estimate);
static void update_pose_from_ekf(slam_system_t *system);
static int32_t calculate_certainty(robot_pose_t *pose_estimate,
                                   robot_model_t *robot_model);

void slam_system_init(slam_system_t *system, slam_system_params_t *params,
                      robot_model_t *robot_model) {
  system->params = *params;
  robot_pose_init(&system->pose);
  map_init(&system->map);

  // float x0[3] = {0};
  // float P0[3][3] = {0};
  // for (int i = 0; i < 3; i++) {
  //   P0[i][i] = 1.0f; // initial covariance
  // }
  // ekf_init(&system->ekf, x0, P0, params->odometry_error_x,
  //          params->odometry_error_y, 0.5f, 0.5f, params->odometry_error_r,
  //          0.1f);

  system->key_pose_id = 0;
  system->key_poses = NULL;

  system->robot_model = *robot_model;

  INFO("SLAM system initialized with map size %d and depth %d", MAP_SIZE,
       MAP_DEPTH);
}

void slam_system_process(slam_system_t *system, odometry_t *odometry,
                         scan_t *scan, float dt) {

  // input odometry to the EKF
  float u[3] = {odometry->dx, odometry->dy, odometry->dr};
  float Qu[3][3] = {
      {system->params.odometry_error_x * odometry->dx, 0, 0},
      {0, system->params.odometry_error_y * odometry->dy, 0},
      {0, 0, system->params.odometry_error_r * odometry->dr},
  };
  // ekf_predict(&system->ekf, dt, u, Qu);

  if (should_add_key_pose(system, &system->pose.pose)) {
    robot_pose_t pose_estimate;
    slam_localisation_result_t localisation_result =
        localise(system, scan, &pose_estimate);
    const int32_t certainty =
        calculate_certainty(&pose_estimate, &system->robot_model);
    switch (localisation_result) {
    case LOCALISATION_INITIALISING:
      DEBUG("Localisation initialising, adding scan to map with certainty %d\n",
            certainty);
      break;
    case LOCALISATION_SUCCESSFUL:
      DEBUG("Localisation successful, pose estimate: (%d, %d, %.6f), "
            "error: (%d, %d, %.6f), certainty: %d",
            pose_estimate.pose.x, pose_estimate.pose.y,
            RAD2DEG(pose_estimate.pose.r), pose_estimate.error.x,
            pose_estimate.error.y, RAD2DEG(pose_estimate.error.r), certainty);
      float z[3] = {0};
      z[0] = pose_estimate.pose.x;
      z[1] = pose_estimate.pose.y;
      z[2] = pose_estimate.pose.r;

      float R[3][3] = {0};
      R[0][0] = pose_estimate.error.x * pose_estimate.error.x;
      R[1][1] = pose_estimate.error.y * pose_estimate.error.y;
      R[2][2] = pose_estimate.error.r * pose_estimate.error.r;

      // ekf_update(&system->ekf, z, R);

      break;
    case LOCALISATION_FAILED:
      ERROR("Localisation failed, increasing error estimate");
      system->pose.error.x *= 1.5f; // increase error estimate
      system->pose.error.y *= 1.5f;
      system->pose.error.r *= 1.5f;
      return;
    }

    update_pose_from_ekf(system);

    add_key_pose(system, &system->pose);

    map_add_scan(&system->map, scan, &system->pose.pose, system->key_pose_id,
                 certainty);
  } else {
    update_pose_from_ekf(system);
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
    return true; // no key poses yet
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
  INFO("Added key pose %zu at (%d, %d, %.6f) with error (%d, %d, %.6f)",
       system->key_pose_id - 1, pose->pose.x, pose->pose.y, pose->pose.r,
       pose->error.x, pose->error.y, pose->error.r);
}

static slam_localisation_result_t localise(slam_system_t *system, scan_t *scan,
                                           robot_pose_t *estimate) {
  // if not enough data to localise yet, just add the scan to the map
  float entropy = map_entropy(&system->map);
  printf("Map entropy: %.6f\n", entropy);
  if (entropy < 0.0002f * MAP_LEAF_SIZE) {
    return LOCALISATION_INITIALISING;
  }

  if (scan_matching_match(scan, &system->robot_model.lidar, &system->map,
                          &system->pose.pose, estimate,
                          system->params.scan_matching_iterations) &&
      !should_relocalise(&system->params, &system->pose, estimate)) {

    return LOCALISATION_SUCCESSFUL;
  } else {
    INFO("Standard scan matching failed, trying relocalisation");
    return relocalise(system, scan, estimate);
  }

  return LOCALISATION_FAILED;
}

static bool should_relocalise(slam_system_params_t *params,
                              robot_pose_t *current_pose,
                              robot_pose_t *estimated_pose) {
  float dist_t = hypotf(current_pose->pose.x - estimated_pose->pose.x,
                        current_pose->pose.y - estimated_pose->pose.y);
  float dist_r = fabsf(current_pose->pose.r - estimated_pose->pose.r);
  return (dist_t > params->relocalise_distance_t ||
          dist_r > params->relocalise_distance_r);
}

static slam_localisation_result_t
relocalise(slam_system_t *system, scan_t *scan, robot_pose_t *estimate) {
  if (course_to_fine_scan_matching_match(scan, &system->map, UINT16_MAX,
                                         &system->pose, estimate)) {
    return LOCALISATION_SUCCESSFUL;
  }
  return LOCALISATION_FAILED;
}

static void update_pose_from_ekf(slam_system_t *system) {
  float out_x[3];
  float out_P[3][3];
  ekf_get_state(&system->ekf, out_x, out_P);

  system->pose.pose.x = (int16_t)out_x[0];
  system->pose.pose.y = (int16_t)out_x[1];
  system->pose.pose.r = out_x[2];

  system->pose.error.x = (int16_t)(out_P[0][0] + out_P[1][1]);
  system->pose.error.y = (int16_t)(out_P[0][0] + out_P[1][1]);
  system->pose.error.r = out_P[2][2];

  DEBUG("EKF updated robot pose: (%d, %d, %.6f), error: (%d, %d, %.6f)",
        system->pose.pose.x, system->pose.pose.y, RAD2DEG(system->pose.pose.r),
        system->pose.error.x, system->pose.error.y,
        RAD2DEG(system->pose.error.r));
}

static int32_t calculate_certainty(robot_pose_t *pose_estimate,
                                   robot_model_t *robot_model) {
  if (pose_estimate->error.x == 0 && pose_estimate->error.y == 0 &&
      pose_estimate->error.r < 1e-5f) {
    return INT32_MAX; // very high certainty
  }

  int32_t error = pose_estimate->error.x * pose_estimate->error.x +
                  pose_estimate->error.y * pose_estimate->error.y +
                  robot_model->lidar.range_error * cosf(pose_estimate->pose.r) +
                  robot_model->lidar.range_error * sinf(pose_estimate->pose.r);

  return MAX(1, MAP_LEAF_SIZE / (error + 1)); // inverse of error, capped at 1
}