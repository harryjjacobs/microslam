#include "wifi.h"

#include <string.h>

#include "esp_wifi.h"
#include "logging.h"
#include "nvs_flash.h"

void wifi_ap_init(void) {
  // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html#wi-fi-configuration-phase
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = WIFI_SSID,
              .ssid_len = strlen(WIFI_SSID),
              .channel = 1,
              .password = WIFI_PASSWORD,
              .max_connection = 2,
              .authmode = WIFI_AUTH_WPA_WPA2_PSK,
          },
  };

  if (strlen(WIFI_PASSWORD) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  INFO("WiFi AP started. SSID:%s password:%s", WIFI_SSID, WIFI_PASSWORD);
}

void wifi_ap_deinit(void) {
  ESP_ERROR_CHECK(esp_wifi_stop());
  ESP_ERROR_CHECK(esp_wifi_deinit());
  ESP_ERROR_CHECK(esp_event_loop_delete_default());
  ESP_ERROR_CHECK(esp_netif_deinit());
  INFO("WiFi AP stopped.");
}
