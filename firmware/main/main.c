#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

// microslam includes
#include "logging.h"
#include "platform.h"
#include "state.h"
#include "system.h"

#define UPDATE_PERIOD_TICKS pdMS_TO_TICKS(50)  // 20 Hz

void app_main(void) {
  platform_init_comms();

  slam_system_t system;
  slam_system_init(&system);

  scan_t scan;
  scan_init(&scan, 0, 0);

  int start, elapsed;
  while (1) {
    start = xTaskGetTickCount();

    INFO("Hello!");

    // perceive
    platform_get_lidar(&scan);

    // process
    // slam_system_process(&system, &scan);

    elapsed = xTaskGetTickCount() - start;
    vTaskDelay(UPDATE_PERIOD_TICKS - elapsed);
  }
}
