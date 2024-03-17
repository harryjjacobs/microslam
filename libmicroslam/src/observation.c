#include <microslam/observation.h>

void observations_init(observations_t *observations) {
  observations->observations = NULL;
  observations->size = 0;
}

void observations_add(observations_t *observations, observation_t observation) {
  observations->observations =
      realloc(observations->observations,
              sizeof(observation_t) * (observations->size + 1));
  if (observations->observations == NULL) {
    fprintf(stderr, "failed to reallod\n");
    exit(EXIT_FAILURE);
  }
  observations->observations[observations->size] = observation;
  observations->size++;
}
