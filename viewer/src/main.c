#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "net.h"
#include "slam/logging.h"
#include "slam/scan.h"
#include "slam/serialisation.h"
#include "slam/viewer.h"

#define MAP_PORT 24601
#define FRAME_TIME_NS 10000000L  // ~100 Hz

static volatile int running = 1;

static pose_t pose;
static occupancy_quadtree_t map;
static scan_t scan;

void sigint_handler(int signo) {
  (void)signo;
  running = 0;
}

static void sleep_remaining_time(struct timespec start_time) {
  struct timespec end_time;
  clock_gettime(CLOCK_MONOTONIC, &end_time);

  long elapsed_ns = (end_time.tv_sec - start_time.tv_sec) * 1000000000L +
                    (end_time.tv_nsec - start_time.tv_nsec);
  long remaining_ns = FRAME_TIME_NS - elapsed_ns;

  if (remaining_ns > 0) {
    struct timespec sleep_time = {
        .tv_sec = remaining_ns / 1000000000L,
        .tv_nsec = remaining_ns % 1000000000L,
    };
    nanosleep(&sleep_time, NULL);
  }
}

bool can_process_message(const char id[4]) {
  if (strncmp(id, SLAM_SERIALISATION_ID_QUADTREE, 4) == 0) {
    return true;
  } else if (strncmp(id, SLAM_SERIALISATION_ID_SCAN, 4) == 0) {
    return true;
  } else {
    return false;
  }
}

void process_message(const char id[4], const serialisation_buffer_t *msg) {
  if (strncmp(id, SLAM_SERIALISATION_ID_QUADTREE, 4) == 0) {
    DEBUG("Received map data of length %zu",
          msg->length - SLAM_SERIALISATION_HEADER_LEN);
    deserialise_quadtree(msg, &(size_t){SLAM_SERIALISATION_HEADER_LEN}, &map);
  } else if (strncmp(id, SLAM_SERIALISATION_ID_SCAN, 4) == 0) {
    DEBUG("Received scan data of length %zu",
          msg->length - SLAM_SERIALISATION_HEADER_LEN);
    deserialise_scan(msg, &(size_t){SLAM_SERIALISATION_HEADER_LEN}, &scan);
  } else {
    ERROR("Unknown message ID: %.4s", id);
  }
}

int main(int argc, char **argv) {
  const char *server_ip = "192.168.4.1";
  if (argc > 1) server_ip = argv[1];

  signal(SIGINT, sigint_handler);

  slam_viewer_t viewer;
  slam_viewer_init(&viewer);

  // initialise the data
  occupancy_quadtree_init(&map, 0.0f, 0.0f, 1.0f, 1);
  scan_reset(&scan);
  pose_init(&pose);

  while (!glfwWindowShouldClose(viewer.window) && running) {
    struct timespec frame_start;
    clock_gettime(CLOCK_MONOTONIC, &frame_start);

    while (!is_connected() && running) {
      INFO("Attempting to connect to %s:%d...", server_ip, MAP_PORT);
      connect_to_server(server_ip, MAP_PORT);
      if (is_connected()) {
        INFO("Connected");
      } else {
        usleep(500000);  // wait 500ms before retrying
      }
    }

    handle_streams();

    slam_viewer_begin_draw();
    slam_viewer_draw_occupancy(&map);
    slam_viewer_draw_scan(&scan, &pose, 1.0f, 0.0f, 0.0f);
    slam_viewer_end_draw(&viewer);

    sleep_remaining_time(frame_start);
  }

  disconnect_client();
  slam_viewer_destroy(&viewer);
  return 0;
}
