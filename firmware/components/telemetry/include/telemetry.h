#ifndef MICROSLAM_TELEMETRY_H
#define MICROSLAM_TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>

#include "esp_event_base.h"

typedef struct {
  bool is_active;  // Flag indicating if telemetry is active
  int port;        // Port number for the telemetry stream
  int listen_fd;   // Listening socket file descriptor
  int client_fd;   // Client socket file descriptor
  uint8_t *pending_data;
  size_t pending_len;
  size_t sent_so_far;
} telemetry_stream_t;

typedef struct {
  telemetry_stream_t *stream;  // Telemetry stream for sending data
  const void *data;            // Pointer to the data to be sent
  size_t len;                  // Length of the data to be sent
} telemetry_send_task_params_t;

/**
 * @brief Initialises an empty telemetry stream structure.
 *
 * @param stream
 */
void telemetry_stream_init(telemetry_stream_t *stream);

/**
 * @brief Initialises the telemetry system. A socket is created for
 * telemetry data transmission and the port is set.
 *
 * @param port Port number for the telemetry stream.
 * @param stream Pointer to the telemetry stream structure to be initialised.
 */
void telemetry_stream_open(int port, telemetry_stream_t *stream);

/**
 * @brief Closes the telemetry stream and releases resources.
 *
 * @param stream
 */
void telemetry_stream_close(telemetry_stream_t *stream);

/**
 * @brief Sends data over the UDP telemetry stream socket.
 *
 * @param stream
 * @param data
 * @param len
 */
void telemetry_send(telemetry_stream_t *stream, const char *data, size_t len);

/**
 * @brief Task to send telemetry data over the network.
 *
 * @param pvParameters
 */
void telemetry_send_task(void *pvParameters);

#endif
