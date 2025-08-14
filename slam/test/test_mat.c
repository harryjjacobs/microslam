#include <math.h>
#include <slam/mat.h>
#include <string.h>
#include <unity/unity.h>

void setUp(void) {}
void tearDown(void) {}

void test_zero6(void) {
  float A[6][6];
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      A[i][j] = 1.0;
  zero6(A);
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      TEST_ASSERT_EQUAL_FLOAT(0.0, A[i][j]);
}

void test_zero3(void) {
  float A[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      A[i][j] = 1.0;
  zero3(A);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      TEST_ASSERT_EQUAL_FLOAT(0.0, A[i][j]);
}

void test_mul6(void) {
  float A[6][6] = {0}, B[6][6] = {0}, C[6][6] = {0};
  for (int i = 0; i < 6; i++)
    A[i][i] = 2.0;
  for (int i = 0; i < 6; i++)
    B[i][i] = 3.0;
  mul6(A, B, C);
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      TEST_ASSERT_EQUAL_FLOAT(i == j ? 6.0 : 0.0, C[i][j]);
}

void test_mul3(void) {
  float A[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  float B[3][3] = {{9, 8, 7}, {6, 5, 4}, {3, 2, 1}};
  float C[3][3];
  mul3(A, B, C);
  TEST_ASSERT_EQUAL_FLOAT(30, C[0][0]);
  TEST_ASSERT_EQUAL_FLOAT(24, C[0][1]);
  TEST_ASSERT_EQUAL_FLOAT(18, C[0][2]);
  TEST_ASSERT_EQUAL_FLOAT(84, C[1][0]);
  TEST_ASSERT_EQUAL_FLOAT(69, C[1][1]);
  TEST_ASSERT_EQUAL_FLOAT(54, C[1][2]);
  TEST_ASSERT_EQUAL_FLOAT(138, C[2][0]);
  TEST_ASSERT_EQUAL_FLOAT(114, C[2][1]);
  TEST_ASSERT_EQUAL_FLOAT(90, C[2][2]);
}

void test_add6(void) {
  float A[6][6] = {0}, B[6][6] = {0}, out[6][6] = {0};
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++) {
      A[i][j] = i + j;
      B[i][j] = 2 * (i + j);
    }
  add6(A, B, out);
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      TEST_ASSERT_EQUAL_FLOAT(3 * (i + j), out[i][j]);
}

void test_add3(void) {
  float A[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  float B[3][3] = {{9, 8, 7}, {6, 5, 4}, {3, 2, 1}};
  float out[3][3];
  add3(A, B, out);
  TEST_ASSERT_EQUAL_FLOAT(10, out[0][0]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[0][1]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[0][2]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[1][0]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[1][1]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[1][2]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[2][0]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[2][1]);
  TEST_ASSERT_EQUAL_FLOAT(10, out[2][2]);
}

void test_transpose6(void) {
  float A[6][6], At[6][6];
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      A[i][j] = i * 6 + j;
  transpose6(A, At);
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      TEST_ASSERT_EQUAL_FLOAT(A[j][i], At[i][j]);
}

void test_transpose3_6(void) {
  float A[3][6], At[6][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 6; j++)
      A[i][j] = i * 6 + j;
  transpose3_6(A, At);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 6; j++)
      TEST_ASSERT_EQUAL_FLOAT(A[i][j], At[j][i]);
}

void test_transpose6_3(void) {
  float A[6][3], At[3][6];
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++)
      A[i][j] = i * 3 + j;
  transpose6_3(A, At);
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++)
      TEST_ASSERT_EQUAL_FLOAT(A[i][j], At[j][i]);
}

void test_inv3(void) {
  float A[3][3] = {{4, 7, 2}, {3, 6, 1}, {2, 5, 1}};
  float expected[3][3] = {{1.0f / 3.0f, 1.0f, -5.0f / 3.0f},
                          {-1.0f / 3.0f, 0.0f, 2.0f / 3.0f},
                          {1.0f, -2.0f, 1.0f}};
  float Ainv[3][3];
  int ret = inv3(A, Ainv);
  TEST_ASSERT_EQUAL_INT(0, ret);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      TEST_ASSERT_FLOAT_WITHIN(1e-6, expected[i][j], Ainv[i][j]);
  // Singular matrix
  float S[3][3] = {{1, 2, 3}, {2, 4, 6}, {3, 6, 9}};
  ret = inv3(S, Ainv);
  TEST_ASSERT_EQUAL_INT(-1, ret);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_zero6);
  RUN_TEST(test_zero3);
  RUN_TEST(test_mul6);
  RUN_TEST(test_mul3);
  RUN_TEST(test_add6);
  RUN_TEST(test_add3);
  RUN_TEST(test_transpose6);
  RUN_TEST(test_transpose3_6);
  RUN_TEST(test_transpose6_3);
  RUN_TEST(test_inv3);
  return UNITY_END();
}
