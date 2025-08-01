#include <GLFW/glfw3.h>
#include <math.h>
#include <slam/logging.h>
#include <slam/occupancy_quadtree.h>
#include <slam/viewer.h>
#include <stdio.h>
#include <stdlib.h>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800

#define WORLD_SIZE 5000.0f  // size of the world in meters

static void key_callback(GLFWwindow *window, int key, int scancode, int action,
                         int mods) {
  (void)scancode;
  (void)mods;
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void slam_viewer_draw_robot(pose_t *pose, float r, float g, float b, float a) {
  static float size = 50;
  glPushMatrix();
  glTranslatef(pose->x, pose->y, 0);
  glRotatef(pose->r * 180 / PI - 90.0f, 0, 0, 1);
  glBegin(GL_TRIANGLES);
  glColor4f(r, g, b, a);
  glVertex2f(0, size);
  glVertex2f(size, -size);
  glVertex2f(-size, -size);
  glEnd();
  glPopMatrix();
}

void slam_viewer_draw_error(pose_t *pose, pose_t *error, float r, float g,
                            float b, float a) {
  // draw covariance 90% confidence region ellipse
  float a_x = error->x * 2.4477f;  // 90% confidence interval for x
  float a_y = error->y * 2.4477f;  // 90% confidence interval for y
  float angle = pose->r;
  glPushMatrix();
  glTranslatef(pose->x, pose->y, 0);
  glRotatef(angle * 180 / PI, 0, 0, 1);
  glBegin(GL_LINE_LOOP);
  glColor4f(r, g, b, a);
  for (int i = 0; i < 360; i++) {
    float theta = DEG2RAD(i);
    float x = a_x * cosf(theta);
    float y = a_y * sinf(theta);
    glVertex2f(x, y);
  }
  glEnd();
  glPopMatrix();
}

void window_resize_callback(GLFWwindow *window, int width, int height) {
  (void)window;
  // Keep the viewport square
  int viewport_size = fmin(width, height);
  int x_offset = (width - viewport_size) / 2;
  int y_offset = (height - viewport_size) / 2;

  // Set the viewport to be square and centered
  glViewport(x_offset, y_offset, viewport_size, viewport_size);

  // Set up an orthographic projection centered at (0,0)
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-WORLD_SIZE / 2, WORLD_SIZE / 2, -WORLD_SIZE / 2, WORLD_SIZE / 2,
          -1.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
}

void slam_viewer_init(slam_viewer_t *viewer) {
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    exit(EXIT_FAILURE);
  }
  viewer->window =
      glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "microSLAM", NULL, NULL);
  if (!viewer->window) {
    glfwTerminate();
    fprintf(stderr, "Failed to create window\n");
    exit(EXIT_FAILURE);
  }
  glfwSetWindowSizeCallback(viewer->window, window_resize_callback);
  glfwSetKeyCallback(viewer->window, key_callback);
  glfwSetInputMode(viewer->window, GLFW_STICKY_KEYS, GLFW_TRUE);
  glfwMakeContextCurrent(viewer->window);
  window_resize_callback(viewer->window, WINDOW_WIDTH, WINDOW_HEIGHT);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glClearColor(1, 1, 1, 1);
}

void slam_viewer_begin_draw() { glClear(GL_COLOR_BUFFER_BIT); }

void slam_viewer_end_draw(slam_viewer_t *viewer) {
  glfwSwapBuffers(viewer->window);
  glfwPollEvents();
}

void slam_viewer_draw_occupancy_leaf(occupancy_quadtree_t *leaf, void *data) {
  (*(size_t *)data)++;
  glPushMatrix();
  glTranslatef(leaf->x - leaf->size / 2.0f, leaf->y - leaf->size / 2.0f, 0);
  glBegin(GL_QUADS);
  if (leaf->occupancy == OCCUPANCY_OCCUPIED) {
    glColor4f(0, 0, 0, 1.0f - 1.0f / (1.0f + expf(leaf->log_odds)));
  } else if (leaf->occupancy == OCCUPANCY_FREE) {
    glColor3f(1, 1, 1);
  } else {
    fprintf(stderr, "Leaf node should not have mixed occupancy\n");
    exit(EXIT_FAILURE);
  }
  glVertex2f(0, 0);
  glVertex2f(leaf->size, 0);
  glVertex2f(leaf->size, leaf->size);
  glVertex2f(0, leaf->size);
  glEnd();
  glPopMatrix();
}

void slam_viewer_draw_occupancy(occupancy_quadtree_t *occupancy) {
  size_t leaf_count = 0;
  occupancy_quadtree_iterate_leafs_depth_first(occupancy, &leaf_count,
                                               slam_viewer_draw_occupancy_leaf);
  // DEBUG("Drew %zu leafs", leaf_count);
}

void slam_viewer_draw_scan(scan_t *scan, pose_t *pose, float r, float g,
                           float b) {
  glBegin(GL_LINES);
  glColor3f(r, g, b);
  for (size_t i = 0; i < 360; i++) {
    glVertex2f(pose->x, pose->y);
    glVertex2f(pose->x + scan->range[i] * cosf(pose->r + DEG2RAD(i)),
               pose->y + scan->range[i] * sinf(pose->r + DEG2RAD(i)));
  }
  glEnd();
}

void slam_viewer_draw_all(occupancy_quadtree_t *occupancy,
                          robot_pose_t *estimated_robot_state,
                          robot_pose_t *gt_robot_state, scan_t *scan) {
  slam_viewer_draw_occupancy(occupancy);
  slam_viewer_draw_scan(scan, &gt_robot_state->pose, 0, 0, 1);
  slam_viewer_draw_robot(&estimated_robot_state->pose, 0, 1, 0, 1);
  slam_viewer_draw_robot(&gt_robot_state->pose, 1, 0, 0, 0.5);
}

void slam_viewer_wait(slam_viewer_t *viewer) {
  while (!glfwWindowShouldClose(viewer->window)) {
    glfwSwapBuffers(viewer->window);
    glfwPollEvents();
  }
}

slam_viewer_key slam_viewer_getkey(slam_viewer_t *viewer) {
  int state;
  state = glfwGetKey(viewer->window, GLFW_KEY_W);
  if (state == GLFW_PRESS) {
    return slam_viewer_key_up;
  }
  state = glfwGetKey(viewer->window, GLFW_KEY_S);
  if (state == GLFW_PRESS) {
    return slam_viewer_key_down;
  }
  state = glfwGetKey(viewer->window, GLFW_KEY_A);
  if (state == GLFW_PRESS) {
    return slam_viewer_key_left;
  }
  state = glfwGetKey(viewer->window, GLFW_KEY_D);
  if (state == GLFW_PRESS) {
    return slam_viewer_key_right;
  }
  return slam_viewer_key_none;
}

void slam_viewer_destroy(slam_viewer_t *viewer) {
  glfwDestroyWindow(viewer->window);
  glfwTerminate();
}
