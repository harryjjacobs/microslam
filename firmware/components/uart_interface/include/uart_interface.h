#ifndef MICROSLAM_UART_INTERFACE_H
#define MICROSLAM_UART_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the UART interface.
 *
 * @param rx_buffer_size Size of the receive buffer in bytes.
 */
void uart_init(int rx_buffer_size);

/**
 * @brief Write data to the UART interface.
 *
 * @param data Pointer to the data to write.
 * @param length Length of the data to write in bytes.
 * @return int Number of bytes written, or a negative error code on failure.
 */
int uart_write(const void *data, uint32_t length);

/**
 * @brief Read data from the UART interface.
 *
 * @param data Pointer to the buffer to store the read data.
 * @param length Length of the buffer in bytes.
 * @return int Number of bytes read, or a negative error code on failure.
 */
int uart_read(void *data, uint32_t length);

#endif  // UART_INTERFACE_H