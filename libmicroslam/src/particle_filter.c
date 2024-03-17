#include <math.h>
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

  particle_t *particles =
      malloc(sizeof(particle_t) * particle_filter->params.num_particles);
  // start from a random point and then jump
  double r = random_uniform();
  double step = 1.0 / particle_filter->params.num_particles;
  double cum_weight = 0.0;
  size_t i = 0;
  for (size_t p = 0; p < particle_filter->params.num_particles; p++) {
    double u = r + p * step;
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

double observation_likelihood(observation_t *observation,
                              observation_t *predicted_observation) {
  double likelihood_bearing =
      normal_pdf(observation->bearing, predicted_observation->bearing,
                 observation->bearing_error);
  double likelihood_range =
      normal_pdf(observation->range, predicted_observation->range,
                 observation->range_error);
  return likelihood_bearing * likelihood_range;
}

/**
 * @brief Update the particles using the motion model
 *
 * @param control
 */
void particle_filter_motion_update(particle_t *particle, motion_t *control) {
  particle->state.pose.x += control->dx + random_normal(0, control->error.x);
  particle->state.pose.y += control->dy + random_normal(0, control->error.y);
  particle->state.pose.r = rotate(particle->state.pose.r, control->dr) +
                           random_normal(0, control->error.r);
}

/**
 * @brief Update the weights from the sensor observation
 *
 * @param observation
 */
void particle_filter_sensor_update(particle_t *particle, landmarks_t *landmarks,
                                   observations_t *observations) {
  // importance weight = the probablity of measurement z under the particle x
  // i.e. p(z | x)
  // assuming we are at particle x, what is the probability we are seeing this
  // observation
  // p(z | x) = p(x | z).p(z)
  // what is the likelihood that we are seeing this observation based on this
  // particle location

  // calculate the predicted measurements - i.e. if we're at this particle, what
  // would the measurement to each of the landmarks be

  // pose_t obs_origin = {0, 0, 0};
  // double observed_range_measurement =
  //     pose_distance(&obs_origin, &observation->delta_pose);
  // // i.e. what would the relative bearing be given we can see the landmark
  // // at this point
  // double observed_bearing_measurement =
  //     calc_bearing_to_point(&obs_origin, &observation->delta_pose);
  // printf("observed_bearing_measurement: %f\n", observed_bearing_measurement);

  double likelihood = 1.0;
  for (size_t i = 0; i < observations->size; i++) {
    pose_t *landmark_pose =
        &landmarks->poses[observations->observations[i].landmark_index];
    // predicted measurement = the euclidean distance between particle and
    // landmark
    // double predicted_range_measurement =
    //     pose_distance(&particle->state.pose, &landmarks->poses[l]);
    // i.e. what would the relative bearing be given we can see the landmark
    // at this point
    // double predicted_bearing_measurement = particle->state.pose.r;
    // printf("predicted_range_measurement: %f\n", predicted_range_measurement);
    // printf("predicted_bearing_measurement: %f\n",
    //        predicted_bearing_measurement);
    // We use the gaussian probability function to calculate the likelihood of
    // getting the given sensor observation, given the predicted measurement.
    // Here we evaluate the sensor measurement using a normal PDF with a mean of
    // predicted measurement and stddev of the sensor. e.g. if the sensor
    // measurement is identical to the expected measurement for this particle
    // and landmark the likelihood will be at maximum
    // double distance_likelihood =
    //     normal_pdf(observed_range_measurement, predicted_range_measurement,
    //                observation->error.x * observation->error.y);
    // double bearing_likelihood =
    //     normal_pdf(observed_bearing_measurement,
    //     predicted_bearing_measurement,
    //                observation->error.r);

    // printf("observed_range_measurement: %f\n", observed_range_measurement);
    // printf("observed_bearing_measurement: %f\n",
    // observed_bearing_measurement);
    // printf("predicted_range_measurement:
    // %f\n", predicted_range_measurement);
    // printf("predicted_bearing_measurement: %f\n",
    //        predicted_bearing_measurement);
    // printf("distance_likelihood: %f\n", distance_likelihood);
    // printf("bearing_likelihood: %f\n", bearing_likelihood);

    observation_t predicted_observation;
    predicted_observation.range =
        pose_distance(landmark_pose, &particle->state.pose);
    predicted_observation.bearing =
        calc_bearing_to_point(&particle->state.pose, landmark_pose);

    likelihood *= observation_likelihood(&observations->observations[i],
                                         &predicted_observation);
  }

  // update the weight for this particle based on the likelihood (we scale it
  // later)
  particle->weight *= likelihood;
  particle->weight += 1.0e-300;  // avoid zero weight
}

void particle_filter_init(particle_filter_t *particle_filter,
                          particle_filter_params_t params) {
  // initialise the particles
  particle_filter->particles =
      malloc(sizeof(particle_t) * params.num_particles);
  particle_filter->params = params;
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    particle_filter->particles[m].weight = 1.0 / params.num_particles;
    particle_filter->particles[m].state.pose.x =
        random_range_uniformf(0.0, (double)MAP_WIDTH);
    particle_filter->particles[m].state.pose.y =
        random_range_uniformf(0.0, (double)MAP_HEIGHT);
    particle_filter->particles[m].state.pose.r =
        random_range_uniformf(0.0, 2 * PI);
  }
}

void particle_filter_step(particle_filter_t *particle_filter, map_t *map,
                          motion_t *control, observations_t *observations) {
  time_t start = time(NULL);
  double weights_sum = 0.0;
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    // update with motion model
    particle_filter_motion_update(&particle_filter->particles[m], control);
    // update weight from sensor observation
    particle_filter_sensor_update(&particle_filter->particles[m],
                                  &map->landmarks, observations);
    weights_sum += particle_filter->particles[m].weight;
  }
  double weights_mean = weights_sum / particle_filter->params.num_particles;
  // normalise the weights and calculate the variance of the weights
  double weights_sum_sq = 0.0;  // sum of the squares of the weights
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    // normalise the weights
    particle_filter->particles[m].weight /= weights_sum;
    // sum of squares of the weights - for calculating the variance
    double diff = (weights_mean - particle_filter->particles[m].weight);
    weights_sum_sq += diff * diff;
  }
  double weights_variance =
      weights_sum_sq / (particle_filter->params.num_particles - 1);
  // use the variance of the importance weights to determine whether
  // or not to perform the resampling. i.e. if all the weights are identical
  // there is no point in resampling, however if they are concentrated on a
  // small set of particles then we should resample - in order to achieve
  // the target probability distribution.
  double variance_threshold =
      (1.0 / particle_filter->params.num_particles) * 0.001;
  // printf("variance_threshold: %f\n", variance_threshold);
  if (weights_variance > variance_threshold) {
    // resampling step
    printf("resampling\n\n");
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
                      (double)particle_filter->params.num_particles);
  // calculate the estimated state variance
  pose_init(&particle_filter->state.error);
  for (size_t m = 0; m < particle_filter->params.num_particles; m++) {
    pose_t diff = pose_subtract(&particle_filter->state.pose,
                                &particle_filter->particles[m].state.pose);
    pose_multiply_inplace_unclamped_rot(&diff, &diff);
    pose_add_inplace_unclamped_rot(&particle_filter->state.error, &diff);
  }
  pose_divide_inplace(&particle_filter->state.error,
                      (double)particle_filter->params.num_particles);
  time_t end = time(NULL);
  double elapsed = difftime(end, start);
  printf("particle_filter_step took %f seconds\n", elapsed);
  printf("state: \nx: %f\ny: %f\nr: %f\n", particle_filter->state.pose.x,
         particle_filter->state.pose.y, particle_filter->state.pose.r);
  printf("variance: \nx: %f\ny: %f\nr: %f\n", particle_filter->state.error.x,
         particle_filter->state.error.y, particle_filter->state.error.r);
}

void particle_filter_destroy(particle_filter_t *particle_filter) {
  free(particle_filter->particles);
}
