#include <log/log.h>
#include <math.h>
#include <microslam/map.h>
#include <microslam/microslam_viewer.h>
#include <microslam/observation.h>
#include <microslam/particle_filter.h>

#define PARTICLE_COUNT 5000

void get_observations(map_t *map, robot_t *robot,
                      observations_t *observations) {
  for (size_t i = 0; i < map->landmarks.size; i++) {
    // check if the landmark is in the robot's FOV
    double min_bearing = -robot->sensor.fov / 2;
    double max_bearing = robot->sensor.fov / 2;
    double bearing =
        calc_bearing_to_point(&robot->state.pose, &map->landmarks.poses[i]);
    double range = pose_distance(&map->landmarks.poses[i], &robot->state.pose);
    if (bearing > min_bearing && bearing < max_bearing &&
        range < robot->sensor.range) {
      log_debug("robot->state.pose.r %f\n", robot->state.pose.r);
      log_debug("min_bearing: %f\n", min_bearing);
      log_debug("max_bearing: %f\n", max_bearing);
      log_debug("bearing: %f\n", bearing);
      log_debug("range: %f\n", range);
      log_debug("observed\n");
      observation_t observation;
      observation.bearing = bearing;
      observation.bearing_error = robot->sensor.bearing_error;
      observation.range = range;
      observation.range_error = robot->sensor.range_error;
      observation.landmark_index = i;
      observations_add(observations, observation);
    }
  }
}

void move_robot(robot_t *robot, motion_t *motion) {
  robot->state.pose.x += motion->dx;
  robot->state.pose.y += motion->dy;
  robot->state.pose.r = rotate(robot->state.pose.r, motion->dr);
}

int main() {
  log_set_level(LOG_DEBUG);

  microslam_viewer_t viewer;
  viewer_init(&viewer);

  particle_filter_params_t particle_filter_params;
  particle_filter_params.num_particles = PARTICLE_COUNT;

  particle_filter_t particle_filter;
  particle_filter_init(&particle_filter, particle_filter_params);

  map_t map;
  map_init(&map);

  // map_add_landmark(&map, (pose_t){10, 10, 0});
  map_add_landmark(&map, (pose_t){MAP_WIDTH / 2 + 1, MAP_HEIGHT / 2 + 1, 0}, 0);
  map_add_landmark(&map, (pose_t){MAP_WIDTH / 2 + 5, MAP_HEIGHT / 2, 0}, 1);
  map_add_landmark(&map, (pose_t){MAP_WIDTH / 2 + 2, MAP_HEIGHT / 2 + 1.1, 0},
                   1);
  map_add_landmark(&map, (pose_t){MAP_WIDTH / 2 + 0.2, MAP_HEIGHT / 2 + 1, 0},
                   1);

  robot_t robot;
  robot.state.pose.x = MAP_WIDTH / 2;
  robot.state.pose.y = MAP_HEIGHT / 2;
  robot.state.pose.r = 0;

  robot.sensor.range = 2;
  robot.sensor.fov = PI / 6;
  robot.sensor.range_error = 0.1;
  robot.sensor.bearing_error = 0.1;

  {
    // motion_t motion;
    // motion.dx = 0;
    // motion.dy = 0;
    // motion.dr = 0;
    // motion.error.x = 0.3;
    // motion.error.y = 0.3;
    // motion.error.r = 0.1;

    // observations_t observations;
    // observations_init(&observations);

    // observation_t observation_1;
    // observation_1.signature = 0;
    // observation_1.signature_error = 0.1;
    // observation_1.bearing = -PI / 4;
    // observation_1.bearing_error = 0.1;
    // observation_1.range = 1;
    // observation_1.range_error = 0.1;
    // observation_1.landmark_index = 0;

    // observations_add(&observations, observation_1);

    // observation_t observation_2;
    // observation_2.signature = 1;
    // observation_2.signature_error = 0.1;
    // observation_2.bearing = -PI / 2;
    // observation_2.bearing_error = 0.1;
    // observation_2.range = 5;
    // observation_2.range_error = 0.1;
    // observation_2.landmark_index = 1;

    // observations_add(&observations, observation_2);

    // particle_filter_step(&particle_filter, &map, &motion, &observation1);
    // particle_filter_step(&particle_filter, &map, &motion, &observation2);
    // particle_filter_step(&particle_filter, &map, &motion, &observation);
    // particle_filter_step(&particle_filter, &map, &motion, &observation);

    while (!glfwWindowShouldClose(viewer.window)) {
      // process input
      motion_t motion;
      motion.dx = 0;
      motion.dy = 0;
      motion.dr = 0;
      motion.error.x = 0.002;
      motion.error.y = 0.002;
      motion.error.r = 0.002;
      double linear_speed = 0.01;
      double angular_speed = 0.02;
      microslam_viewer_key key = viewer_getkey(&viewer);
      switch (key) {
        case microslam_viewer_key_up:
          motion.dx = -(linear_speed + random_normal(0, motion.error.x)) *
                      cos(robot.state.pose.r - PI / 2);
          motion.dy = -(linear_speed + random_normal(0, motion.error.y)) *
                      sin(robot.state.pose.r - PI / 2);
          break;
        case microslam_viewer_key_down:
          motion.dx = (linear_speed + random_normal(0, motion.error.x)) *
                      cos(robot.state.pose.r - PI / 2);
          motion.dy = (linear_speed + random_normal(0, motion.error.y)) *
                      sin(robot.state.pose.r - PI / 2);
          break;
        case microslam_viewer_key_left:
          motion.dr = angular_speed + random_normal(0, motion.error.r);
          break;
        case microslam_viewer_key_right:
          motion.dr = -angular_speed + random_normal(0, motion.error.r);
          break;
        default:
          break;
      }

      observations_t observations;
      observations_init(&observations);

      if (key != microslam_viewer_key_none) {
        move_robot(&robot, &motion);

        get_observations(&map, &robot, &observations);

        particle_filter_step(&particle_filter, &map, &motion, &observations);
      }

      // draw
      viewer_draw(&viewer, &particle_filter, &map, &robot, &observations);
    }
  }

  return 0;
}