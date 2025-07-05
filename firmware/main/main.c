// clang-format off
#include "logging.h"
// clang-format on

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

// microslam includes

#include <string.h>

#include "platform.h"
#include "slam/serialisation.h"
#include "state.h"
#include "system.h"
#include "telemetry.h"
#include "wifi.h"

#define TELEM_MAP_PORT (24601)  // Port for telemetry map stream

#define UPDATE_PERIOD_TICKS (pdMS_TO_TICKS(50))  // 20 Hz

void send_map(telemetry_stream_t *telem_stream, slam_system_t *system) {
  serialisation_buffer_t buffer;
  serialisation_buffer_init(&buffer);

  if (write_header(&buffer, SLAM_SERIALISATION_ID_QUADTREE) < 0) {
    ERROR("Failed to write header");
  }

  if (serialise_quadtree(&system->map, &buffer) < 0) {
    ERROR("Failed to serialise quadtree");
    return;
  }

  if (write_header_length(&buffer) < 0) {
    ERROR("Failed to write length into header");
  }

  telemetry_send(telem_stream, buffer.data, buffer.length);
  serialisation_buffer_free(&buffer);
}

void send_scan(telemetry_stream_t *telem_stream, scan_t *scan) {
  serialisation_buffer_t buffer;
  serialisation_buffer_init(&buffer);

  if (write_header(&buffer, SLAM_SERIALISATION_ID_SCAN) < 0) {
    ERROR("Failed to write header");
  }

  if (serialise_scan(scan, &buffer) < 0) {
    ERROR("Failed to serialise scan");
    return;
  }

  if (write_header_length(&buffer) < 0) {
    ERROR("Failed to write length into header");
  }

  telemetry_send(telem_stream, buffer.data, buffer.length);
  serialisation_buffer_free(&buffer);
}

void app_main(void) {
  wifi_ap_init();

  telemetry_stream_t telem_stream;
  telemetry_stream_init(&telem_stream);
  telemetry_stream_open(TELEM_MAP_PORT, &telem_stream);

  slam_system_t system;
  slam_system_init(&system);

  scan_t scan;
  scan_reset(&scan);

  platform_init_comms();

  int start, elapsed;
  while (1) {
    start = xTaskGetTickCount();

    INFO("Hello!");

    // perceive
    platform_get_lidar(&scan);

    // process
    slam_system_process(&system, &scan);

    // send telemetry
    send_map(&telem_stream, &system);
    send_scan(&telem_stream, &scan);

    // check heap memory usage
    uint32_t free_heap = esp_get_free_heap_size();
    INFO("Free heap memory: %u bytes", free_heap);

    elapsed = xTaskGetTickCount() - start;
    vTaskDelay(UPDATE_PERIOD_TICKS - elapsed);
  }
}
