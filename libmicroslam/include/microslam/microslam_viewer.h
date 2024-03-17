#ifndef MICROSLAM_VIEWER_H_
#define MICROSLAM_VIEWER_H_

#include <GLFW/glfw3.h>
#include <microslam/types.h>
#include <microslam/utils.h>

typedef enum microslam_viewer_key {
  microslam_viewer_key_up,
  microslam_viewer_key_down,
  microslam_viewer_key_left,
  microslam_viewer_key_right,
  microslam_viewer_key_none
} microslam_viewer_key;

typedef struct microslam_viewer_t {
  GLFWwindow *window;
} microslam_viewer_t;

void viewer_init(microslam_viewer_t *viewer);
void viewer_draw(microslam_viewer_t *viewer, particle_filter_t *particle_filter,
                 map_t *map, robot_t *robot, observations_t *observations);
void viewer_wait(microslam_viewer_t *viewer);
microslam_viewer_key viewer_getkey(microslam_viewer_t *viewer);
void viewer_destroy(microslam_viewer_t *viewer);

#endif