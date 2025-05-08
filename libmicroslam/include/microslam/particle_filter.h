/*
 * microslam.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harryjjacobs
 */

#ifndef MICROSLAM_PARTICLE_FILTER_H_
#define MICROSLAM_PARTICLE_FILTER_H_

#include "types.h"
#include "utils.h"

void particle_filter_low_variance_resampling(
    particle_filter_t *particle_filter);

void particle_filter_init(particle_filter_t *particle_filter,
                          particle_filter_params_t params);

/**
 * @brief Special function to update the particles with the motion model.
 * Normally this is done in the particle_filter_step function, but
 * it may be useful to call this function directly if you want to
 * update the particle motion without updating the weights.
 *
 * @param particle_filter
 * @param control
 */
void particle_filter_motion_update(particle_filter_t *particle_filter,
                                   motion_t *control);

/**
 * @brief The main function of the particle filter. This function
 * updates the particles with the motion model and then updates
 * the weights of the particles using the sensor model.
 *
 * This function also performs the resampling step if the
 * variance of the weights is above a certain threshold.
 *
 * The function also calculates the estimated state of the
 * robot by taking the mean of the particles.
 *
 * After calling this function, the state of the particle filter
 * will be updated with the estimated state of the robot.
 *
 * @param particle_filter
 * @param occ
 * @param control
 * @param lidar
 * @param scan
 */
void particle_filter_step(particle_filter_t *particle_filter,
                          occupancy_quadtree_t *occ, motion_t *control,
                          lidar_sensor_t *lidar, scan_t *scan);

/**
 * @brief Free the memory used internally by the particle filter.
 *
 * @param particle_filter
 */
void particle_filter_destroy(particle_filter_t *particle_filter);

#endif /* MICROSLAM_PARTICLE_FILTER_H_ */
