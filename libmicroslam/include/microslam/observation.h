/*
 * microslam.h
 *
 *  Created on: Mar 5, 2023
 *      Author: harryjjacobs
 */

#ifndef MICROSLAM_OBSERVATION_H_
#define MICROSLAM_OBSERVATION_H_

#include <stdio.h>
#include <stdlib.h>

#include "types.h"

void observations_init(observations_t *observations);

void observations_add(observations_t *observations, observation_t observation);

#endif /* MICROSLAM_OBSERVATION_H_ */
