#include <math.h>
#include <slam/mat.h>
#include <string.h>
#include <unity/unity.h>

void setUp(void) {}
void tearDown(void) {}

void test_zero3(void) {
  float A[9];
  for (int i = 0; i < 9; i++)
    A[i] = 1.0;
  zero3(A);
  for (int i = 0; i < 9; i++)
    TEST_ASSERT_EQUAL_FLOAT(0.0, A[i]);
}

void test_mul3(void) {
  float A[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  float B[9] = {9, 8, 7, 6, 5, 4, 3, 2, 1};
  float C[9];
  mul3(A, B, C);
  TEST_ASSERT_EQUAL_FLOAT(30, C[0]);
  TEST_ASSERT_EQUAL_FLOAT(24, C[1]);
  TEST_ASSERT_EQUAL_FLOAT(18, C[2]);
  TEST_ASSERT_EQUAL_FLOAT(84, C[3]);
  TEST_ASSERT_EQUAL_FLOAT(69, C[4]);
  TEST_ASSERT_EQUAL_FLOAT(54, C[5]);
  TEST_ASSERT_EQUAL_FLOAT(138, C[6]);
  TEST_ASSERT_EQUAL_FLOAT(114, C[7]);
  TEST_ASSERT_EQUAL_FLOAT(90, C[8]);
}

void test_add3(void) {
  float A[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  float B[9] = {9, 8, 7, 6, 5, 4, 3, 2, 1};
  float out[9];
  add3(A, B, out);
  TEST_ASSERT_EQUAL_FLOAT(10, out[0]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[1]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[2]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[3]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[4]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[5]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[6]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[7]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[8]);
}

void test_inv3(void) {
  float A[9] = {4, 7, 2, 3, 6, 1, 2, 5, 1};
  float expected[9] = {1.0f / 3.0f, 1.0f, -5.0f / 3.0f, -1.0f / 3.0f, 0.0f,
                       2.0f / 3.0f, 1.0f, -2.0f,        1.0f};
  float Ainv[9];
  int ret = inv3(A, Ainv);
  TEST_ASSERT_EQUAL_INT(0, ret);
  for (int i = 0; i < 9; i++)
    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected[i], Ainv[i]);
  // Singular matrix
  float S[9] = {1, 2, 3, 2, 4, 6, 3, 6, 9};
  ret = inv3(S, Ainv);
  TEST_ASSERT_EQUAL_INT(-1, ret);
}

void test_transpose3(void) {
  float A[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  float expected[9] = {1, 4, 7, 2, 5, 8, 3, 6, 9};
  float B[9];
  transpose3(A, B);
  for (int i = 0; i < 9; i++)
    TEST_ASSERT_EQUAL_FLOAT(expected[i], B[i]);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_zero3);
  RUN_TEST(test_mul3);
  RUN_TEST(test_add3);
  RUN_TEST(test_inv3);
  RUN_TEST(test_transpose3);
  return UNITY_END();
}
