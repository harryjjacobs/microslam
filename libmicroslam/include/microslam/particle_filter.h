/*
 * microslam.h
 *
 *  Created on: Mar 5, 2023
 *      Author: harryjjacobs
 */

#ifndef MICROSLAM_PARTICLE_FILTER_H_
#define MICROSLAM_PARTICLE_FILTER_H_

#include "types.h"
#include "utils.h"

void particle_filter_low_variance_resampling(
    particle_filter_t *particle_filter);

double particle_filter_observation_likelihood(
    observations_t *observation, observations_t *predicted_observation);

void particle_filter_motion_update(particle_t *particle, motion_t *control);

void particle_filter_sensor_update(particle_t *particle, landmarks_t *landmarks,
                                   observations_t *observations);

void particle_filter_init(particle_filter_t *particle_filter,
                          particle_filter_params_t params);

void particle_filter_step(particle_filter_t *particle_filter, map_t *map,
                          motion_t *control, observations_t *observations);

void particle_filter_destroy(particle_filter_t *particle_filter);

#endif /* MICROSLAM_PARTICLE_FILTER_H_ */
