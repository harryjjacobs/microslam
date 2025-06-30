#include "led.h"

#include "led_strip.h"

static led_strip_handle_t strip;

void led_init() {
  led_strip_config_t strip_cfg = {
      .strip_gpio_num = 38,
      .max_leds = 1,
  };
  led_strip_rmt_config_t rmt_cfg = {
      .resolution_hz = 10 * 1000 * 1000,  // 10MHz
      .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip));
}

void led_toggle() {
  static bool s_led_state = false;

  if (s_led_state) {
    led_strip_set_pixel(strip, 0, 0, 255, 0);  // red
    led_strip_refresh(strip);
  } else {
    led_strip_clear(strip);
  }
  s_led_state = !s_led_state;
}
