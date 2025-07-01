#include "platform.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_compiler.h"
#include "esp_err.h"
#include "esp_log.h"
#include "logging.h"
#include "uart_interface.h"

static void platform_set_testmode(bool enable) {
  const char *cmd = enable ? "TestMode On\n" : "TestMode Off\n";
  size_t len = strlen(cmd);

  int written = uart_write(cmd, len);
  if (unlikely(written < 0)) {
    FATAL("uart_write failed");
  }
}

static void platform_set_lds_rotation(bool enable) {
  const char *cmd = enable ? "SetLDSRotation On\n" : "SetLDSRotation Off\n";
  size_t len = strlen(cmd);

  int written = uart_write(cmd, len);
  if (unlikely(written < 0)) {
    FATAL("uart_write failed");
  }
}

void platform_init_comms(void) {
  uart_init(PLATFORM_UART_BUF_SIZE);
  platform_set_testmode(true);
  platform_set_lds_rotation(true);
}

void platform_get_lidar(scan_t *scan) {
  static const char *cmd = "GetLDSScan\n";
  size_t len = strlen(cmd);

  // request laser scan data
  int bytes = uart_write(cmd, len);
  if (unlikely(bytes <= 0)) {
    FATAL("uart_write failed");
  }

  char buffer[PLATFORM_UART_BUF_SIZE];
  size_t buffer_len = 0;

  int angle = 0, range;

  // block until we receive the first character
  while ((buffer_len = uart_read(buffer, 1)) <= 0) {
    // wait for data
  }

  char line[48];  // temporary line buffer
  while (angle < 359) {
    // read data into buffer
    int bytes_read =
        uart_read(buffer + buffer_len, PLATFORM_UART_BUF_SIZE - buffer_len - 1);
    if (bytes_read <= 0) {
      continue;
    }
    buffer_len += bytes_read;
    buffer[buffer_len] = '\0';  // null-terminate to use string functions

    // process line by line
    char *start = buffer;
    char *newline;
    while ((newline = strchr(start, '\n')) != NULL) {
      size_t line_len = newline - start;
      if (line_len >= sizeof(line)) {
        FATAL("Line too long: %.*s", (int)line_len, start);
      }

      memcpy(line, start, line_len);
      line[line_len] = '\0';

      // Parse line
      if (sscanf(line, "%d,%d", &angle, &range) == 2) {
        scan->range[scan->hits] = range / 1000.0;
        scan->hits++;
      }

      // move to the next line
      start = newline + 1;
    }

    // shift the remaining data to the start of the buffer
    buffer_len = strlen(start);
    memmove(buffer, start, buffer_len);
  }

  DEBUG("Lidar scan complete: %d hits", scan->hits);
}

void platform_set_motor_wheels_enable(bool left_enable, bool right_enable) {
  char cmd[38];
  snprintf(cmd, sizeof(cmd), "SetMotor %s %s\n",
           left_enable ? "LWheelEnable" : "LWheelDisable",
           right_enable ? "RWheelEnable" : "RWheelDisable");
  size_t len = strlen(cmd);

  int written = uart_write(cmd, len);
  if (unlikely(written < 0)) {
    FATAL("uart_write failed");
  }
}

void platform_set_motor_wheels(short left_dist, short right_dist, short speed,
                               short accel) {
  char cmd[72];
  snprintf(cmd, sizeof(cmd),
           "SetMotor LWheelDist %d RWheelDist %d Speed %d Accel %d\n",
           left_dist, right_dist, speed, accel);
  size_t len = strlen(cmd);

  int written = uart_write(cmd, len);
  if (unlikely(written < 0)) {
    FATAL("uart_write failed");
  }
}
