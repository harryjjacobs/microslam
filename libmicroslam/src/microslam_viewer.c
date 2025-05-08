#include <GLFW/glfw3.h>
#include <math.h>
#include <microslam/microslam_viewer.h>
#include <stdio.h>
#include <stdlib.h>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800

static void key_callback(GLFWwindow *window, int key, int scancode, int action,
                         int mods) {
  (void)scancode;
  (void)mods;
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void draw_robot(pose_t *pose) {
  static float size = 0.2;
  glPushMatrix();
  glTranslatef(pose->x, pose->y, 0);
  glRotatef(pose->r * 180 / PI, 0, 0, 1);
  glBegin(GL_TRIANGLES);
  glColor3f(1, 0, 0);
  glVertex2f(0, size);
  glVertex2f(size, -size);
  glVertex2f(-size, -size);
  glEnd();
  glPopMatrix();
}

void draw_particles(particle_t *particles, size_t num_particles) {
  for (size_t i = 0; i < num_particles; i++) {
    float size = 0.05;
    glPushMatrix();
    glTranslatef(particles[i].state.pose.x, particles[i].state.pose.y, 0);
    glRotatef(particles[i].state.pose.r * 180 / PI, 0, 0, 1);
    glBegin(GL_TRIANGLES);
    glColor3f(0, 0, 1);
    glVertex2f(0, size);
    glVertex2f(size, -size);
    glVertex2f(-size, -size);
    glEnd();
    glPopMatrix();
  }
}

void draw_state(state_t *state) {
  static float size = 0.2;
  glPushMatrix();
  glTranslatef(state->pose.x, state->pose.y, 0);
  glRotatef(state->pose.r * 180 / PI, 0, 0, 1);
  glBegin(GL_TRIANGLES);
  glColor3f(1, 0, 1);
  glVertex2f(0, size);
  glVertex2f(size, -size);
  glVertex2f(-size, -size);
  glEnd();
  glPopMatrix();
}

// void draw_map(map_t *map) {
//   for (size_t x = 0; x < MAP_WIDTH; x++) {
//     for (size_t y = 0; y < MAP_HEIGHT; y++) {
//       if (map->map[x][y] == 1) {
//         glPushMatrix();
//         glTranslatef(x, y, 0);
//         glBegin(GL_QUADS);
//         glColor3f(0, 0, 0);
//         glVertex2f(0, 0);
//         glVertex2f(1, 0);
//         glVertex2f(1, 1);
//         glVertex2f(0, 1);
//         glEnd();
//         glPopMatrix();
//       }
//     }
//   }
// }

void draw_observations(pose_t *pose, observations_t *observations) {
  for (size_t i = 0; i < observations->size; i++) {
    glPushMatrix();
    glTranslatef(pose->x, pose->y, 0);
    glRotatef((pose->r + observations->observations[i].bearing) * 180 / PI, 0,
              0, 1);
    glBegin(GL_LINES);
    glColor3f(1, 1, 0);
    glVertex2f(0, 0);
    glVertex2f(0, observations->observations[i].range);
    glEnd();
    glPopMatrix();
  }
}

void viewer_init(microslam_viewer_t *viewer) {
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
  glfwSetKeyCallback(viewer->window, key_callback);
  glfwSetInputMode(viewer->window, GLFW_STICKY_KEYS, GLFW_TRUE);
  glfwMakeContextCurrent(viewer->window);
  glOrtho(0, MAP_WIDTH, MAP_HEIGHT, 0, -1, 1);
  glClearColor(1, 1, 1, 1);
}

void viewer_draw(microslam_viewer_t *viewer, particle_filter_t *particle_filter,
                 map_t *map, robot_t *robot, observations_t *observations) {
  glClear(GL_COLOR_BUFFER_BIT);
  // draw_map(map);
  // draw_landmarks(&map->landmarks);
  draw_robot(&robot->state.pose);
  draw_observations(&robot->state.pose, observations);
  draw_particles(particle_filter->particles,
                 particle_filter->params.num_particles);
  draw_state(&particle_filter->state);
  glfwSwapBuffers(viewer->window);
  glfwPollEvents();
}

void viewer_wait(microslam_viewer_t *viewer) {
  while (!glfwWindowShouldClose(viewer->window)) {
    glfwSwapBuffers(viewer->window);
    glfwPollEvents();
  }
}

microslam_viewer_key viewer_getkey(microslam_viewer_t *viewer) {
  int state;
  state = glfwGetKey(viewer->window, GLFW_KEY_W);
  if (state == GLFW_PRESS) {
    return microslam_viewer_key_up;
  }
  state = glfwGetKey(viewer->window, GLFW_KEY_S);
  if (state == GLFW_PRESS) {
    return microslam_viewer_key_down;
  }
  state = glfwGetKey(viewer->window, GLFW_KEY_A);
  if (state == GLFW_PRESS) {
    return microslam_viewer_key_left;
  }
  state = glfwGetKey(viewer->window, GLFW_KEY_D);
  if (state == GLFW_PRESS) {
    return microslam_viewer_key_right;
  }
  return microslam_viewer_key_none;
}

void viewer_destroy(microslam_viewer_t *viewer) {
  glfwDestroyWindow(viewer->window);
  glfwTerminate();
}
