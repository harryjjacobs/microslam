#include <log/log.h>
#include <math.h>
#include <microslam/occupancy_quadtree.h>
#include <microslam/particle_filter.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void particle_filter_low_variance_resampling(
    particle_filter_t *particle_filter) {
  // randomly resample the particles according to the weights.
  // benefits of low variance sampling:
  // - more systematic than picking a bunch of samples at random
  // - no samples lost if there was no observation update/weight change
  // - fast

  particle_t *particles = (particle_t *)malloc(
      sizeof(particle_t) * particle_filter->params.num_particles);
  // start from a random point and then jump
  float r = random_uniform();
  float step = 1.0 / particle_filter->params.num_particles;
  float cum_weight = 0.0;  //
  size_t i = 0;
  for (size_t p = 0; p < particle_filter->params.num_particles; p++) {
    float u = r + p * step;
    while (u > cum_weight) {
      i = (i + 1) % particle_filter->params.num_particles;
      cum_weight += particle_filter->particles[i].weight;
    }
    // we've found the weight that resides under this sample point.
    // add it to our particle set
    particles[p] = particle_filter->particles[i];
  }
  free(particle_filter->particles);
  particle_filter->particles = particles;
}

float observation_likelihood(float observation, float predicted_observation,
                             float error) {
  float likelihood = normal_pdf(observation, predicted_observation, error);
  return likelihood;
}

/**
 * @brief Update a particle using the motion model
 *
 * @param control
 */
void particle_filter_particle_motion_update(particle_t *particle,
                                            motion_t *control,
                                            particle_filter_params_t *params) {
  particle->state.pose.x += control->dx + random_normalf(0, control->error.x);
  particle->state.pose.y += control->dy + random_normalf(0, control->error.y);
  particle->state.pose.r =
      clamp_rotation(rotate(particle->state.pose.r, control->dr) +
                     random_normalf(0, control->error.r));
  // wrap particle position to be within the map
  particle->state.pose.x =
      fminf(fmaxf(particle->state.pose.x, -particle->state.pose.x),
            particle->state.pose.x);
  particle->state.pose.y =
      fminf(fmaxf(particle->state.pose.y, -particle->state.pose.y),
            particle->state.pose.y);
}

/**
 * @brief Update the weights of a particle from the sensor observation
 *
 * @param observation
 */
void particle_filter_sensor_update(particle_t *particle,
                                   occupancy_quadtree_t *occupancy_quadtree,
                                   lidar_sensor_t *lidar, scan_t *scan) {
  // // importance weight = the probablity of measurement z under the particle x
  // // i.e. p(z | x)
  // // assuming we are at particle x, what is the probability we are seeing
  // this
  // // observation
  // // p(z | x) = p(x | z).p(z)
  // // what is the likelihood that we are seeing this observation based on this
  // // particle location

  // // calculate the predicted measurements - i.e. if we're at this particle,
  // // what would the measurement to each of the landmarks be
  // float likelihood = 1.0;
  // for (size_t i = 0; i < scan->size; i++) {
  //   pose_t *landmark_pose =
  //       &landmarks->poses[observations->observations[i].landmark_index];
  //   // We use the gaussian probability function to calculate the likelihood
  //   of
  //   // getting the given sensor observation, given the predicted measurement.
  //   // Here we evaluate the sensor measurement using a normal PDF with a mean
  //   // of predicted measurement and stddev of the sensor. e.g. if the sensor
  //   // measurement is identical to the expected measurement for this particle
  //   // and landmark the likelihood will be at maximum
  //   observation_t predicted_observation;
  //   predicted_observation.range =
  //       pose_distance(landmark_pose, &particle->state.pose);
  //   predicted_observation.bearing =
  //       calc_bearing_to_point(&particle->state.pose, landmark_pose);

  //   likelihood *= observation_likelihood(&observations->observations[i],
  //                                        &predicted_observation);
  // }

  // // update the weight for this particle based on the likelihood (we scale it
  // // later)
  // particle->weight *= likelihood;
  // particle->weight += 1.0e-300;  // avoid zero weight

  // calculate the predicted measurements - i.e. if we're at this particle,
  // what would a scan look like (based on the current occupancy quadtree)
  float likelihood = 1.0;
  for (unsigned short i = 0; i < 360;
       i += 5) {  // only use every 5th scan to speed up
    float r = particle->state.pose.r + DEG2RAD(i);
    float dx = cosf(r);
    float dy = sinf(r);

    float pred_range;
    occupancy_quadtree_t *hit = occupancy_quadtree_raycast(
        occupancy_quadtree, particle->state.pose.x, particle->state.pose.y, dx,
        dy, lidar->max_range, &pred_range);
    if (hit == NULL) {
      pred_range = 0;
      continue;
    }
    if (scan->range[i] < 1e-6 && hit != NULL) {
      continue;
    }
    float l =
        observation_likelihood(scan->range[i], pred_range, lidar->range_error);
    // l = fmaxf(l, 1e-6);  // avoid zero likelihood
    likelihood *= l;
  }

  // update the weight for this particle based on the likelihood (we scale it
  // later)
  particle->weight *= likelihood;
  particle->weight += 1.0e-300;  // avoid zero weight

  // printf("likelihood: %f\n", likelihood);
}

void particle_filter_init(particle_filter_t *particle_filter,
                          particle_filter_params_t params) {
  // initialise the particles
  particle_filter->particles =
      (particle_t *)malloc(sizeof(particle_t) * params.num_particles);
  particle_filter->params = params;
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    particle_filter->particles[m].weight = 1.0 / params.num_particles;
    particle_filter->particles[m].state.pose.x =
        random_range_uniformf(-params.map_width / 2, params.map_width / 2);
    particle_filter->particles[m].state.pose.y =
        random_range_uniformf(-params.map_height / 2, params.map_height / 2);
    float min_rotation = params.initial_rotation - params.random_rotation_range;
    float max_rotation = params.initial_rotation + params.random_rotation_range;
    particle_filter->particles[m].state.pose.r =
        random_range_uniformf(min_rotation, max_rotation);
  }
}

void particle_filter_motion_update(particle_filter_t *particle_filter,
                                   motion_t *control) {
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    particle_filter_particle_motion_update(&particle_filter->particles[m],
                                           control, &particle_filter->params);
  }
}

void particle_filter_step(particle_filter_t *particle_filter,
                          occupancy_quadtree_t *occ, motion_t *control,
                          lidar_sensor_t *lidar, scan_t *scan) {
  time_t start = time(NULL);

  // As mentioned in  the paper:
  // "In practice, we have found it useful to add a small number of uniformly
  // distributed, random samples after each estimation step."
  // "The added samples are essential for relocalization in the rare event
  // that the robot loses track of its position. Since MCL uses finite sample
  // sets, it may happen that no sample is generated close to the correct
  // robot position. In such cases, MCL would be unable to re-localize the
  // robot. By adding a small number of random samples, however, MCL can
  // effectively re-localize the robot, as documented in the experimental
  // results section of this paper."

  // add a small number of random particles
  size_t num_random_particles = particle_filter->params.num_particles / 10;
  for (size_t m = 0; m < num_random_particles; m++) {
    particle_t particle;
    particle.weight = 1.0 / particle_filter->params.num_particles;
    particle.state.pose.x =
        random_range_uniformf(-particle_filter->params.map_width / 2,
                              particle_filter->params.map_width / 2);
    particle.state.pose.y =
        random_range_uniformf(-particle_filter->params.map_height / 2,
                              particle_filter->params.map_height / 2);
    particle.state.pose.r = random_range_uniformf(0.0, 2 * PI);
    particle_filter->particles[m] = particle;
  }

  float weights_sum = 0.0;
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    // update with motion model
    particle_filter_particle_motion_update(&particle_filter->particles[m],
                                           control, &particle_filter->params);
    // update weight from sensor observation
    particle_filter_sensor_update(&particle_filter->particles[m], occ, lidar,
                                  scan);
    weights_sum += particle_filter->particles[m].weight;
  }
  float weights_mean = weights_sum / particle_filter->params.num_particles;

  // normalise the weights and calculate the variance of the weights
  float weights_sum_sq = 0.0;  // sum of the squares of the weights
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    // normalise the weights
    particle_filter->particles[m].weight /= weights_sum;
    // sum of squares of the weights - for calculating the variance
    float diff = (weights_mean - particle_filter->particles[m].weight);
    weights_sum_sq += diff * diff;
  }
  float weights_variance =
      weights_sum_sq / (particle_filter->params.num_particles - 1);

  log_debug("weights_variance: %f\n", weights_variance);

  // use the variance of the importance weights to determine whether
  // or not to perform the resampling. i.e. if all the weights are identical
  // there is no point in resampling, however if they are concentrated on a
  // small set of particles then we should resample - in order to achieve
  // the target probability distribution.
  float variance_threshold = 0.0001 / particle_filter->params.num_particles;
  if (weights_variance > variance_threshold) {
    // resampling step
    log_debug("resampling\n\n");
    particle_filter_low_variance_resampling(particle_filter);
  }

  // calculate the estimated state (mean of the particles)
  pose_init(&particle_filter->state.pose);
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    // reset the weights
    particle_filter->particles[m].weight =
        1.0 / particle_filter->params.num_particles;
    // sum the poses
    pose_add_inplace_unclamped_rot(&particle_filter->state.pose,
                                   &particle_filter->particles[m].state.pose);
  }

  // mean
  pose_divide_inplace(&particle_filter->state.pose,
                      (float)particle_filter->params.num_particles);

  // calculate the estimated state variance
  pose_init(&particle_filter->state.error);
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    pose_t diff = pose_subtract(&particle_filter->state.pose,
                                &particle_filter->particles[m].state.pose);
    pose_multiply_inplace_unclamped_rot(&diff, &diff);
    pose_add_inplace_unclamped_rot(&particle_filter->state.error, &diff);
  }
  pose_divide_inplace(&particle_filter->state.error,
                      (float)particle_filter->params.num_particles);

  time_t end = time(NULL);
  float elapsed = difftime(end, start);
  log_debug("particle_filter_step took %f seconds\n", elapsed);
  log_debug("state: \nx: %f\ny: %f\nr: %f\n", particle_filter->state.pose.x,
            particle_filter->state.pose.y, particle_filter->state.pose.r);
  log_debug("variance: \nx: %f\ny: %f\nr: %f\n", particle_filter->state.error.x,
            particle_filter->state.error.y, particle_filter->state.error.r);
}

void particle_filter_destroy(particle_filter_t *particle_filter) {
  free(particle_filter->particles);
}
