#include <slam/logging.h>
#include <slam/occupancy_quadtree.h>
#include <slam/scan.h>
#include <slam/serialisation.h>
#include <slam/types.h>
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
  serialisation_buffer_t buffer;
  serialisation_buffer_init(&buffer);

  // Write header
  int result = write_header(&buffer, &SLAM_SERIALISATION_ID_QUADTREE[0]);
  TEST_ASSERT_EQUAL(0, result);
  TEST_ASSERT_NOT_NULL(buffer.data);
  TEST_ASSERT_EQUAL(buffer.length, SLAM_SERIALISATION_HEADER_LEN);

  // Add some extra data to the buffer
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 16, 3);

  serialise_quadtree(&quadtree, &buffer);
  TEST_ASSERT_EQUAL(33, buffer.length);

  // Write length
  result = write_header_length(&buffer);
  TEST_ASSERT_EQUAL(0, result);

  // Read header
  char id[SLAM_SERIALISATION_ID_LEN];
  uint16_t length = 0;
  result = read_header(&buffer, 0, id, &length);
  TEST_ASSERT_EQUAL(0, result);
  TEST_ASSERT_EQUAL_MEMORY(&SLAM_SERIALISATION_ID_QUADTREE[0], id,
                           SLAM_SERIALISATION_ID_LEN);
  TEST_ASSERT_EQUAL(buffer.length, length);

  serialisation_buffer_free(&buffer);
}

void test_serialise_deserialise_occupancy_quadtree(void) {
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 16, 3);  // leaf size = 2
  occupancy_quadtree_update(&quadtree, -7.5, -7.5, 0, 1);
  occupancy_quadtree_update(&quadtree, 7.5, 7.5, 0, 2);
  occupancy_quadtree_update(&quadtree, -7.5, 7.5, 0, 3);
  occupancy_quadtree_update(&quadtree, 5.5, -7.5, 0, 4);

  serialisation_buffer_t serialised_data;
  serialisation_buffer_init(&serialised_data);
  serialise_quadtree(&quadtree, &serialised_data);

  TEST_ASSERT_NOT_NULL(serialised_data.data);
  TEST_ASSERT_GREATER_THAN(0, serialised_data.length);

  occupancy_quadtree_t deserialised_quadtree;

  int result = deserialise_quadtree(&serialised_data, &(size_t){0},
                                    &deserialised_quadtree);
  TEST_ASSERT_EQUAL(0, result);
  treequal(&quadtree, &deserialised_quadtree);

  occupancy_quadtree_clear(&quadtree);
}

void test_serialise_deserialise_scan(void) {
  scan_t scan;
  scan_reset(&scan);

  for (int i = 0; i < 360; i++) {
    scan.range[i] = (float)i;
    scan.hits++;
  }

  serialisation_buffer_t serialised_data;
  serialisation_buffer_init(&serialised_data);
  int result = serialise_scan(&scan, &serialised_data);
  TEST_ASSERT_EQUAL(0, result);
  TEST_ASSERT_NOT_NULL(serialised_data.data);
  TEST_ASSERT_GREATER_THAN(0, serialised_data.length);
  TEST_ASSERT_EQUAL(sizeof(scan.hits) + 360 * sizeof(float),
                    serialised_data.length);
  TEST_ASSERT_EQUAL(360, scan.hits);
  for (int i = 0; i < 360; i++) {
    TEST_ASSERT_EQUAL_FLOAT((float)i, scan.range[i]);
  }
  scan_t deserialised_scan;
  size_t offset = 0;
  result = deserialise_scan(&serialised_data, &offset, &deserialised_scan);
  TEST_ASSERT_EQUAL(0, result);
  TEST_ASSERT_EQUAL(scan.hits, deserialised_scan.hits);
  TEST_ASSERT_EQUAL_MEMORY(scan.range, deserialised_scan.range,
                           sizeof(scan.range));

  serialisation_buffer_free(&serialised_data);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_write_read_header);
  RUN_TEST(test_serialise_deserialise_occupancy_quadtree);
  RUN_TEST(test_serialise_deserialise_scan);

  UNITY_END();
  return 0;
}
