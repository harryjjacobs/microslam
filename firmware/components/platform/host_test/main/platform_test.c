#include "platform.h"

#include "Mockuart_interface.h"
#include "slam/scan.h"
#include "string.h"
#include "unity.h"

static string_t lidar_scan_str = {0};
static size_t lidar_scan_str_pos = 0;

void generate_lidar_scan_str(string_t *lidar_scan_str) {
  string_init(lidar_scan_str);
  for (int angle = 0; angle < 360; angle++) {
    char buffer[32];
    // angle, distance, intensity, error
    snprintf(buffer, sizeof(buffer), "%d,%d,%d,%d\n", angle, 2 * angle,
             3 * angle, 0);
    string_append(lidar_scan_str, buffer);
  }
}

void test_platform_init_comms(void) {
  uart_init_Expect(PLATFORM_UART_BUF_SIZE);
  uart_write_ExpectAndReturn("TestMode On\n", 12, 12);
  uart_write_ExpectAndReturn("SetLDSRotation On\n", 18, 18);
  platform_init_comms();
}

int uart_read_callback(void *data, uint32_t length, int cmock_num_calls) {
  if (data == NULL || length == 0) {
    return -1;
  }
  if (lidar_scan_str_pos >= lidar_scan_str.length) {
    return 0;  // No more data to read
  }
  size_t remaining = lidar_scan_str.length - lidar_scan_str_pos;
  size_t to_read = (remaining < length) ? remaining : length;
  memcpy(data, lidar_scan_str.data + lidar_scan_str_pos, to_read);
  lidar_scan_str_pos += to_read;
  return to_read;
}

void test_platform_get_lidar(void) {
  scan_t scan;
  scan_reset(&scan);

  uart_write_ExpectAndReturn("GetLDSScan\n", 11, 11);

  // mock the UART read function to return a predefined lidar scan string
  generate_lidar_scan_str(&lidar_scan_str);
  lidar_scan_str_pos = 0;
  uart_read_Stub(uart_read_callback);

  // read the lidar scan
  platform_get_lidar(&scan);

  // clean up the string
  string_free(&lidar_scan_str);

  TEST_ASSERT_EQUAL(360, scan.hits);
  for (int i = 0; i < 360; i++) {
    TEST_ASSERT_EQUAL_FLOAT((i * 2) / 1000.0f, scan.range[i]);
  }
}

void test_platform_set_motor_wheels_enable(void) {
  const char *expected_cmd_1 = "SetMotor LWheelDisable RWheelEnable\n";
  const char *expected_cmd_2 = "SetMotor LWheelEnable RWheelDisable\n";

  uart_write_ExpectAndReturn(expected_cmd_1, strlen(expected_cmd_1),
                             strlen(expected_cmd_1));

  platform_set_motor_wheels_enable(false, true);

  uart_write_ExpectAndReturn(expected_cmd_2, strlen(expected_cmd_2),
                             strlen(expected_cmd_2));

  platform_set_motor_wheels_enable(true, false);
}

void test_platform_set_motor_wheels(void) {
  const char *expected_cmd =
      "SetMotor LWheelDist 1000 RWheelDist 2000 Speed 300 Accel 400\n";

  uart_write_ExpectAndReturn(expected_cmd, strlen(expected_cmd),
                             strlen(expected_cmd));

  platform_set_motor_wheels(1000, 2000, 300, 400);
}

int app_main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_platform_init_comms);
  RUN_TEST(test_platform_get_lidar);
  RUN_TEST(test_platform_set_motor_wheels_enable);
  RUN_TEST(test_platform_set_motor_wheels);
  exit(UNITY_END());
}
