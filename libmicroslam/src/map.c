#include <microslam/map.h>
#include <stdio.h>
#include <stdlib.h>

void map_init(map_t *map) {
  for (int j = 0; j < MAP_HEIGHT; j++) {
    for (int i = 0; i < MAP_WIDTH; i++) {
      map->map[j][i] = 0;
    }
  }
  map->landmarks.poses = NULL;
  map->landmarks.signatures = NULL;
  map->landmarks.size = 0;
}

void map_add_landmark(map_t *map, pose_t pose, int signature) {
  map->landmarks.poses =
      realloc(map->landmarks.poses, sizeof(pose_t) * (map->landmarks.size + 1));
  if (map->landmarks.poses == NULL) {
    fprintf(stderr, "failed to realloc\n");
    exit(EXIT_FAILURE);
  }

  map->landmarks.signatures = realloc(map->landmarks.signatures,
                                      sizeof(int) * (map->landmarks.size + 1));
  if (map->landmarks.signatures == NULL) {
    fprintf(stderr, "failed to realloc\n");
    exit(EXIT_FAILURE);
  }

  map->landmarks.poses[map->landmarks.size] = pose;
  map->landmarks.signatures[map->landmarks.size] = signature;
  map->landmarks.size++;
}

void map_destroy(map_t *map) { (void)map; }