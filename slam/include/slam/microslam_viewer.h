#ifndef MICROSLAM_VIEWER_H_
#define MICROSLAM_VIEWER_H_

#include <GLFW/glfw3.h>
#include <slam/types.h>
#include <slam/utils.h>

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
void viewer_begin_draw();
void viewer_end_draw(microslam_viewer_t *viewer);
void viewer_draw_scan(scan_t *scan, pose_t *pose, float r, float g, float b);
void viewer_draw_occupancy(occupancy_quadtree_t *occupancy);
void viewer_draw_all(occupancy_quadtree_t *occupancy,
                     robot_pose_t *estimated_robot_state,
                     robot_pose_t *gt_robot_state, scan_t *scan);
void viewer_wait(microslam_viewer_t *viewer);
microslam_viewer_key viewer_getkey(microslam_viewer_t *viewer);
void viewer_destroy(microslam_viewer_t *viewer);

#endif