#include <slam/logging.h>
#include <slam/serialisation.h>
#include <unity/unity.h>

#define FLOAT_EPSILON 1e-6f

void treequal(const occupancy_quadtree_t *a, const occupancy_quadtree_t *b) {
  TEST_ASSERT_NOT_NULL(a);
  TEST_ASSERT_NOT_NULL(b);
  TEST_ASSERT_EQUAL_CHAR(a->max_depth, b->max_depth);
  TEST_ASSERT_EQUAL_FLOAT(a->size, b->size);
  TEST_ASSERT_EQUAL_FLOAT(a->x, b->x);
  TEST_ASSERT_EQUAL_FLOAT(a->y, b->y);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, a->log_odds, b->log_odds);
  TEST_ASSERT_EQUAL(a->occupancy, b->occupancy);
  TEST_ASSERT_EQUAL(a->depth, b->depth);
  for (int i = 0; i < 4; i++) {
    if (a->children[i] == NULL && b->children[i] == NULL) continue;
    TEST_ASSERT_NOT_NULL(a->children[i]);
    TEST_ASSERT_NOT_NULL(b->children[i]);
    treequal(a->children[i], b->children[i]);
  }
}

void test_write_read_header(void) {
  char *buffer = NULL;
  size_t buffer_pos = 0;

  // Write header
  int result =
      write_header(&buffer, &buffer_pos, SLAM_SERIALISATION_ID_QUADTREE);
  TEST_ASSERT_EQUAL(0, result);
  TEST_ASSERT_NOT_NULL(buffer);
  TEST_ASSERT_EQUAL(buffer_pos, SLAM_SERIALISATION_ID_LEN + 2);

  // Add some extra data to the buffer
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 16, 3);
  serialise_quadtree(&quadtree, &buffer, &buffer_pos);
  TEST_ASSERT_EQUAL(buffer_pos,
                    SLAM_SERIALISATION_ID_LEN + 2 + sizeof(quadtree));

  // Write length
  result = write_header_length(&buffer, buffer_pos);
  TEST_ASSERT_EQUAL(0, result);

  // Read header
  char id[SLAM_SERIALISATION_ID_LEN];
  size_t length = 0;
  result = read_header(buffer, buffer_pos, id, &length);
  TEST_ASSERT_EQUAL(0, result);
  TEST_ASSERT_EQUAL_MEMORY(SLAM_SERIALISATION_ID_QUADTREE, id,
                           SLAM_SERIALISATION_ID_LEN);
  TEST_ASSERT_EQUAL(buffer_pos, length);

  free(buffer);
}

void test_serialise_deserialise_occupancy_quadtree_init(void) {
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 16, 3);  // leaf size = 2
  occupancy_quadtree_update(&quadtree, -7.5, -7.5, 1);
  occupancy_quadtree_update(&quadtree, 7.5, 7.5, 2);
  occupancy_quadtree_update(&quadtree, -7.5, 7.5, 3);
  occupancy_quadtree_update(&quadtree, 5.5, -7.5, 4);

  char *serialised_data = NULL;
  size_t serialised_size = 0;
  serialise_quadtree(&quadtree, &serialised_data, &serialised_size);

  TEST_ASSERT_NOT_NULL(serialised_data);
  TEST_ASSERT_GREATER_THAN(0, serialised_size);

  occupancy_quadtree_t deserialised_quadtree;

  int result = deserialise_quadtree(serialised_data, serialised_size,
                                    &deserialised_quadtree);
  TEST_ASSERT_EQUAL(0, result);
  treequal(&quadtree, &deserialised_quadtree);

  occupancy_quadtree_clear(&quadtree);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_write_read_header);
  RUN_TEST(test_serialise_deserialise_occupancy_quadtree_init);

  UNITY_END();
  return 0;
}
