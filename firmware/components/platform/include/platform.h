#ifndef MICROSLAM_PLATFORM_H
#define MICROSLAM_PLATFORM_H

#include <stdbool.h>

// microslam includes
#include "slam/types.h"

#define PLATFORM_UART_BUF_SIZE (512)

/**
 * @brief Initialize the platform communication interface.
 *
 * Opens the UART interface, commands the vehicle to enter test mode,
 * and enables the lidar rotation.
 */
void platform_init_comms(void);

/**
 * @brief Reads the lidar scan data from the platform.
 *
 * @param scan Scan structure to populate with the lidar data.
 */
void platform_get_lidar(scan_t *scan);

/**
 * @brief Enable or disable the wheel motors.
 *
 * @param left_enable Enable or disable the left wheel motor.
 * @param right_enable Enable or disable the right wheel motor.
 */
void platform_set_motor_wheels_enable(bool left_enable, bool right_enable);

/**
 * @brief Sets the specified wheel motors to run a certain distance at a given
 * speed and acceleration.
 *
 * @param left_dist Distance for the left wheel motor in mm.
 * @param right_dist Distance for the right wheel motor in mm.
 * @param speed Speed for the wheel motors in mm/s.
 * @param accel Acceleration for the wheel motors in mm/s^2.
 */
void platform_set_motor_wheels(short left_dist, short right_dist, short speed,
                               short accel);

#endif
