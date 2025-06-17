/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "microslam";

// For ESP-IDF v5+ example
#define BLINK_PERIOD 5000

static led_strip_handle_t strip;
static bool s_led_state = false;

void app_main(void) {
  led_strip_config_t strip_cfg = {
      .strip_gpio_num = 38,
      .max_leds = 1,
  };
  led_strip_rmt_config_t rmt_cfg = {
      .resolution_hz = 10 * 1000 * 1000,  // 10MHz
      .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip));

  while (1) {
    ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");

    char c = getchar();

    ESP_LOGI(TAG, "Received character: %c", c);

    if (s_led_state) {
      led_strip_set_pixel(strip, 0, 0, 255, 0);  // red
      led_strip_refresh(strip);
    } else {
      led_strip_clear(strip);
    }
    s_led_state = !s_led_state;
    vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD));
  }
}
