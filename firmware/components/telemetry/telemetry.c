#include "telemetry.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include "esp_wifi.h"
#include "logging.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

void telemetry_stream_init(telemetry_stream_t *stream) {
  stream->is_active = false;
  stream->port = 0;
  stream->listen_fd = -1;
  stream->client_fd = -1;
  stream->pending_data = NULL;
  stream->pending_len = 0;
  stream->sent_so_far = 0;
}

static void set_non_blocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    FATAL("Failed to get socket flags: errno %d", errno);
  }
  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    FATAL("Failed to set socket to non-blocking: errno %d", errno);
  }
}

void telemetry_stream_open(int port, telemetry_stream_t *stream) {
  if (stream->is_active) {
    WARN("Telemetry stream is already active, closing existing stream");
    telemetry_stream_close(stream);
  }

  stream->port = port;
  stream->listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (stream->listen_fd < 0) {
    FATAL("Unable to create socket: errno %d", errno);
  }

  set_non_blocking(stream->listen_fd);

  struct sockaddr_in listen_addr;
  listen_addr.sin_family = AF_INET;
  listen_addr.sin_port = htons(port);
  listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  int opt = 1;
  setsockopt(stream->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  if (bind(stream->listen_fd, (struct sockaddr *)&listen_addr,
           sizeof(listen_addr)) < 0) {
    FATAL("Socket bind failed: errno %d", errno);
  }

  if (listen(stream->listen_fd, 1) < 0) {
    FATAL("Socket listen failed: errno %d", errno);
  }

  stream->client_fd = -1;
  stream->is_active = true;

  DEBUG("Telemetry server started on port %d", port);
}

void telemetry_accept_client(telemetry_stream_t *stream) {
  if (!stream->is_active || stream->client_fd >= 0) {
    return;
  }

  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);
  int client_fd =
      accept(stream->listen_fd, (struct sockaddr *)&client_addr, &addr_len);
  if (client_fd < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      ERROR("Accept failed: errno %d", errno);
    }
    return;
  }

  set_non_blocking(client_fd);
  stream->client_fd = client_fd;
  DEBUG("Client connected (fd %d)", client_fd);
}

void telemetry_stream_close(telemetry_stream_t *stream) {
  if (stream->client_fd >= 0) {
    close(stream->client_fd);
    stream->client_fd = -1;
    DEBUG("Client connection closed");
  }
  if (stream->listen_fd >= 0) {
    close(stream->listen_fd);
    stream->listen_fd = -1;
    DEBUG("Listening socket closed");
  }
  if (stream->pending_data) {
    free(stream->pending_data);
    stream->pending_data = NULL;
    stream->pending_len = 0;
    stream->sent_so_far = 0;
  }

  stream->is_active = false;
}

void telemetry_send(telemetry_stream_t *stream, const char *data, size_t len) {
  if (!stream->is_active) return;

  if (stream->client_fd < 0) {
    telemetry_accept_client(stream);
    return;
  }

  // First, flush any pending data
  if (stream->pending_data) {
    size_t to_send = stream->pending_len - stream->sent_so_far;
    ssize_t sent =
        send(stream->client_fd,
             (uint8_t *)stream->pending_data + stream->sent_so_far, to_send, 0);
    if (sent < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        close(stream->client_fd);
        stream->client_fd = -1;
      }
      return;  // Try again later
    }
    stream->sent_so_far += sent;
    if (stream->sent_so_far < stream->pending_len) {
      return;  // Still not done sending pending data
    }

    // All pending data sent
    free(stream->pending_data);
    stream->pending_data = NULL;
    stream->pending_len = 0;
    stream->sent_so_far = 0;
  }

  // Try to send the new data
  ssize_t sent = send(stream->client_fd, data, len, 0);
  if (sent < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // Queue full buffer for later
      stream->pending_data = malloc(len);
      if (!stream->pending_data) FATAL("Failed to allocate memory");
      memcpy(stream->pending_data, data, len);
      stream->pending_len = len;
      stream->sent_so_far = 0;
    } else {
      close(stream->client_fd);
      stream->client_fd = -1;
    }
  } else if ((size_t)sent < len) {
    // Partial send: queue the rest
    size_t remaining = len - sent;
    stream->pending_data = malloc(remaining);
    if (!stream->pending_data) FATAL("Failed to allocate memory");
    memcpy(stream->pending_data, (uint8_t *)data + sent, remaining);
    stream->pending_len = remaining;
    stream->sent_so_far = 0;
  }
}

void telemetry_send_task(void *pvParameters) {
  telemetry_send_task_params_t *params =
      (telemetry_send_task_params_t *)pvParameters;

  telemetry_send(params->stream, params->data, params->len);
  vTaskDelete(NULL);
}
