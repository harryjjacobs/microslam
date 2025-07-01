#include "uart_interface.h"

#include "driver/uart.h"

#define UART_NUM UART_NUM_0

void uart_init(int rx_buffer_size) {
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM, rx_buffer_size, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

int uart_write(const void *data, uint32_t length) {
  return uart_write_bytes(UART_NUM, (const char *)data, length);
}

int uart_read(void *data, uint32_t length) {
  return uart_read_bytes(UART_NUM, data, length, 20 / portTICK_PERIOD_MS);
}