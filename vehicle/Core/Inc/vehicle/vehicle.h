/*
 * vehicle.h
 *
 *  Created on: Mar 5, 2023
 *      Author: harryjjacobs
 */

#ifndef INC_VEHICLE_H_
#define INC_VEHICLE_H_

#include "stm32g4xx_hal.h"

#include "../adc.h"
#include "../tim.h"
#include "../gpio.h"
#include "../usart.h"

#ifdef __cplusplus
extern "C" {
#endif

void vehicle_init();
void vehicle_update();

#ifdef __cplusplus
}
#endif

#endif /* INC_VEHICLE_H_ */
