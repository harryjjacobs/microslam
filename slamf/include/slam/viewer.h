#ifndef SLAM_VIEWER_H_
#define SLAM_VIEWER_H_

#include <GLFW/glfw3.h>
#include <slam/types.h>
#include <slam/utils.h>

typedef enum slam_viewer_key {
  slam_viewer_key_up,
  slam_viewer_key_down,
  slam_viewer_key_left,
  slam_viewer_key_right,
  slam_viewer_key_none
} slam_viewer_key;

typedef struct slam_viewer_t {
  GLFWwindow *window;
} slam_viewer_t;

void slam_viewer_init(slam_viewer_t *viewer);
void slam_viewer_begin_draw();
void slam_viewer_end_draw(slam_viewer_t *viewer);
void slam_viewer_draw_robot(pose_t *pose, float r, float g, float b, float a);
void slam_viewer_draw_scan(scan_t *scan, pose_t *pose, float r, float g,
                           float b);
void slam_viewer_draw_occupancy(occupancy_quadtree_t *occupancy);
void slam_viewer_draw_all(occupancy_quadtree_t *occupancy,
                          robot_pose_t *estimated_robot_state,
                          robot_pose_t *gt_robot_state, scan_t *scan);
void slam_viewer_wait(slam_viewer_t *viewer);
slam_viewer_key slam_viewer_getkey(slam_viewer_t *viewer);
void slam_viewer_destroy(slam_viewer_t *viewer);

#endif