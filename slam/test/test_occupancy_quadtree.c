#include <slam/logging.h>
#include <slam/occupancy_quadtree.h>
#include <unity/unity.h>

#define FLOAT_EPSILON 1e-6f

void test_occupancy_quadtree_init() {
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 1, 0);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0, quadtree.x);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 0, quadtree.y);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPSILON, 1, quadtree.size);
  TEST_ASSERT_EQUAL_INT(0, quadtree.depth);
  TEST_ASSERT_EQUAL_INT(0, quadtree.max_depth);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_FREE, quadtree.occupancy);
  for (int i = 0; i < 4; i++) {
    TEST_ASSERT_NULL(quadtree.children[i]);
  }
  occupancy_quadtree_clear(&quadtree);
}

void test_occupancy_quadtree_update() {
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 1024, 1);

  occupancy_quadtree_update(&quadtree, -200, -200, 1);  // log odds = 1
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED, quadtree.occupancy);
  TEST_ASSERT_TRUE(quadtree.children[0] != NULL);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_OCCUPIED, quadtree.children[0]->occupancy);
  TEST_ASSERT_EQUAL(512, quadtree.children[0]->size);
  TEST_ASSERT_EQUAL(1, quadtree.children[0]->log_odds);
  TEST_ASSERT_TRUE(quadtree.children[1] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[2] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[3] == NULL);

  // ensure that occupancy is updated and nodes are removed when log odds <= 0
  occupancy_quadtree_update(&quadtree, -200, -200, -1);  // log odds = -1
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_FREE, quadtree.occupancy);
  TEST_ASSERT_TRUE(quadtree.children[0] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[1] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[2] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[3] == NULL);

  occupancy_quadtree_clear(&quadtree);
}

void test_occupancy_quadtree_update_multiple() {
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 1024, 1);
  occupancy_quadtree_update(&quadtree, -200, -200, 1);  // log odds = 1
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED, quadtree.occupancy);
  TEST_ASSERT_TRUE(quadtree.children[0] != NULL);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_OCCUPIED, quadtree.children[0]->occupancy);
  TEST_ASSERT_EQUAL(1, quadtree.children[0]->log_odds);

  occupancy_quadtree_update(&quadtree, 200, -200, 2);  // log odds = 2
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED, quadtree.occupancy);
  TEST_ASSERT_TRUE(quadtree.children[1] != NULL);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_OCCUPIED, quadtree.children[1]->occupancy);
  TEST_ASSERT_EQUAL(2, quadtree.children[1]->log_odds);

  occupancy_quadtree_update(&quadtree, -200, 200, 1);  // log odds = 1
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED, quadtree.occupancy);
  TEST_ASSERT_TRUE(quadtree.children[2] != NULL);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_OCCUPIED, quadtree.children[2]->occupancy);
  TEST_ASSERT_EQUAL(1, quadtree.children[2]->log_odds);

  occupancy_quadtree_update(&quadtree, 200, 200, -1);  // log odds = -1
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED, quadtree.occupancy);
  TEST_ASSERT_TRUE(quadtree.children[3] == NULL);

  occupancy_quadtree_clear(&quadtree);
}

void test_occupancy_quadtree_update_depth_2() {
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 16, 2);
  occupancy_quadtree_update(&quadtree, -7.5, -7.5, 1);  // log odds = 1

  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED, quadtree.occupancy);
  TEST_ASSERT_TRUE(quadtree.children[0] != NULL);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED, quadtree.children[0]->occupancy);

  TEST_ASSERT_TRUE(quadtree.children[0]->children[0] != NULL);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_OCCUPIED,
                        quadtree.children[0]->children[0]->occupancy);
  TEST_ASSERT_EQUAL_FLOAT(1.0, quadtree.children[0]->children[0]->log_odds);
  TEST_ASSERT_EQUAL_FLOAT(8, quadtree.children[0]->size);
  TEST_ASSERT_EQUAL_FLOAT(-4, quadtree.children[0]->x);
  TEST_ASSERT_EQUAL_FLOAT(4, quadtree.children[0]->children[0]->size);

  TEST_ASSERT_TRUE(quadtree.children[0]->children[1] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[0]->children[2] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[0]->children[3] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[0]->children[0]->children[0] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[0]->children[0]->children[1] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[0]->children[0]->children[2] == NULL);

  occupancy_quadtree_clear(&quadtree);
}

void test_occupancy_quadtree_update_depth_3() {
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 512, 3);

  occupancy_quadtree_update(&quadtree, 255, 255, 1);  // log odds = 1
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED, quadtree.occupancy);

  // depth 1
  TEST_ASSERT_TRUE(quadtree.children[0] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[1] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[2] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[3] != NULL);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED, quadtree.children[3]->occupancy);
  TEST_ASSERT_EQUAL_FLOAT(256, quadtree.children[3]->size);
  TEST_ASSERT_EQUAL_FLOAT(128, quadtree.children[3]->x);
  TEST_ASSERT_EQUAL_FLOAT(128, quadtree.children[3]->y);
  TEST_ASSERT_EQUAL_FLOAT(0, quadtree.children[3]->log_odds);

  // depth 2
  TEST_ASSERT_TRUE(quadtree.children[3]->children[0] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[3]->children[1] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[3]->children[2] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[3]->children[3] != NULL);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_MIXED,
                        quadtree.children[3]->children[3]->occupancy);
  TEST_ASSERT_EQUAL_FLOAT(128, quadtree.children[3]->children[3]->size);
  TEST_ASSERT_EQUAL_FLOAT(192, quadtree.children[3]->children[3]->x);
  TEST_ASSERT_EQUAL_FLOAT(192, quadtree.children[3]->children[3]->y);
  TEST_ASSERT_EQUAL_FLOAT(0, quadtree.children[3]->children[3]->log_odds);

  // depth 3
  TEST_ASSERT_TRUE(quadtree.children[3]->children[3]->children[0] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[3]->children[3]->children[1] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[3]->children[3]->children[2] == NULL);
  TEST_ASSERT_TRUE(quadtree.children[3]->children[3]->children[3] != NULL);
  TEST_ASSERT_EQUAL_INT(
      OCCUPANCY_OCCUPIED,
      quadtree.children[3]->children[3]->children[3]->occupancy);
  TEST_ASSERT_EQUAL(64, quadtree.children[3]->children[3]->children[3]->size);
  TEST_ASSERT_EQUAL(224, quadtree.children[3]->children[3]->children[3]->x);
  TEST_ASSERT_EQUAL(224, quadtree.children[3]->children[3]->children[3]->y);
  TEST_ASSERT_EQUAL(1,
                    quadtree.children[3]->children[3]->children[3]->log_odds);
}

void test_occupancy_quadtree_find() {
  occupancy_quadtree_t quadtree;
  occupancy_quadtree_init(&quadtree, 0, 0, 1024, 1);
  occupancy_quadtree_update(&quadtree, -200, -200, 1);
  occupancy_quadtree_update(&quadtree, 200, -200, 2);
  occupancy_quadtree_update(&quadtree, -200, 200, 3);
  occupancy_quadtree_update(&quadtree, 200, 200, 4);

  occupancy_quadtree_t *node = occupancy_quadtree_find(&quadtree, -200, -200);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(-256, node->x);
  TEST_ASSERT_EQUAL(-256, node->y);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_OCCUPIED, node->occupancy);

  node = occupancy_quadtree_find(&quadtree, 200, -200);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(256, node->x);
  TEST_ASSERT_EQUAL(-256, node->y);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_OCCUPIED, node->occupancy);

  node = occupancy_quadtree_find(&quadtree, -200, 200);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(-256, node->x);
  TEST_ASSERT_EQUAL(256, node->y);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_OCCUPIED, node->occupancy);

  node = occupancy_quadtree_find(&quadtree, 200, 200);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(256, node->x);
  TEST_ASSERT_EQUAL(256, node->y);
  TEST_ASSERT_EQUAL_INT(OCCUPANCY_OCCUPIED, node->occupancy);

  node = occupancy_quadtree_find(&quadtree, 200, 5000);
  TEST_ASSERT_TRUE(node == NULL);

  occupancy_quadtree_clear(&quadtree);
}

void test_occupancy_quadtree_nearest() {
  occupancy_quadtree_t quadtree;
  // leaf size = 512/2^5 = 16
  occupancy_quadtree_init(&quadtree, 0, 0, 512, 5);
  occupancy_quadtree_update(&quadtree, -17, -17, 1);
  occupancy_quadtree_update(&quadtree, 255, 255, 1);
  occupancy_quadtree_update(&quadtree, 66, -66, 1);

  uint16_t distance = UINT16_MAX;

  occupancy_quadtree_t *node =
      occupancy_quadtree_nearest(&quadtree, 1, 1, &distance);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(-24, node->x);
  TEST_ASSERT_EQUAL(-24, node->y);
  TEST_ASSERT_EQUAL(16, node->size);
  TEST_ASSERT_EQUAL(sqrt((-24 - 1) * (-24 - 1) + (-24 - 1) * (-24 - 1)),
                    distance);

  node = occupancy_quadtree_nearest(&quadtree, -16, -16, &distance);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(-24, node->x);
  TEST_ASSERT_EQUAL(-24, node->y);
  TEST_ASSERT_EQUAL(16, node->size);
  TEST_ASSERT_EQUAL(sqrt((-24 + 16) * (-24 + 16) + (-24 + 16) * (-24 + 16)),
                    distance);

  node = occupancy_quadtree_nearest(&quadtree, 0, 0, &distance);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(-24, node->x);
  TEST_ASSERT_EQUAL(-24, node->y);
  TEST_ASSERT_EQUAL(16, node->size);
  TEST_ASSERT_EQUAL(sqrt((-24) * (-24) + (-24) * (-24)), distance);

  node = occupancy_quadtree_nearest(&quadtree, 600, 600, &distance);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(248, node->x);
  TEST_ASSERT_EQUAL(248, node->y);
  TEST_ASSERT_EQUAL(16, node->size);
  TEST_ASSERT_EQUAL(sqrt((248 - 600) * (248 - 600) + (248 - 600) * (248 - 600)),
                    distance);
}

void test_occupancy_quadtree_raycast() {
  occupancy_quadtree_t quadtree;
  // leaf size = 512/2^5 = 16
  occupancy_quadtree_init(&quadtree, 0, 0, 512, 5);

  occupancy_quadtree_update(&quadtree, -17, -17, 1);
  occupancy_quadtree_update(&quadtree, 255, 255, 1);
  occupancy_quadtree_update(&quadtree, 66, -66, 1);

  occupancy_quadtree_t *node;

  node = occupancy_quadtree_raycast(&quadtree, 0, 0, 1, 1, 100, NULL);
  TEST_ASSERT_TRUE(node == NULL);  // no intersection within max range

  uint16_t distance = UINT16_MAX;
  node = occupancy_quadtree_raycast(&quadtree, 0, 0, -1, -1, 32, &distance);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(-24, node->x);
  TEST_ASSERT_EQUAL(-24, node->y);
  TEST_ASSERT_EQUAL(sqrt((-24 + 8) * (-24 + 8) + (-24 + 8) * (-24 + 8)),
                    distance);

  distance = -1;
  node = occupancy_quadtree_raycast(&quadtree, 0, 0, 1, 1, 600, &distance);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(256 - 8, node->x);
  TEST_ASSERT_EQUAL(256 - 8, node->y);
  TEST_ASSERT_EQUAL(16, node->size);
  TEST_ASSERT_EQUAL(sqrt((256 - 16) * (256 - 16) + (256 - 16) * (256 - 16)),
                    distance);

  distance = -1;
  node = occupancy_quadtree_raycast(&quadtree, 0, 0, 1, -1, 65, &distance);
  TEST_ASSERT_TRUE(node != NULL);
  TEST_ASSERT_EQUAL(72, node->x);
  TEST_ASSERT_EQUAL(-72, node->y);
  TEST_ASSERT_EQUAL(16, node->size);
  TEST_ASSERT_EQUAL(sqrt((72 - 8) * (72 - 8) + (-72 + 8) * (-72 + 8)),
                    distance);

  occupancy_quadtree_clear(&quadtree);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_occupancy_quadtree_init);
  RUN_TEST(test_occupancy_quadtree_update);
  RUN_TEST(test_occupancy_quadtree_update_multiple);
  RUN_TEST(test_occupancy_quadtree_update_depth_2);
  RUN_TEST(test_occupancy_quadtree_update_depth_3);
  RUN_TEST(test_occupancy_quadtree_find);
  RUN_TEST(test_occupancy_quadtree_nearest);
  RUN_TEST(test_occupancy_quadtree_raycast);

  UNITY_END();
  return 0;
}
