#ifndef SLAM_MAT_H
#define SLAM_MAT_H

#include "slam/ekf.h"

/**
 * @brief Set all elements of a 6x6 matrix to zero.
 * @param A The 6x6 matrix to zero.
 */
void zero6(float A[6][6]);

/**
 * @brief Set all elements of a 3x3 matrix to zero.
 * @param A The 3x3 matrix to zero.
 */
void zero3(float A[3][3]);

/**
 * @brief Multiply two 6x6 matrices: C = A * B.
 * @param A First 6x6 matrix.
 * @param B Second 6x6 matrix.
 * @param C Output 6x6 matrix.
 */
void mul6(const float A[6][6], const float B[6][6], float C[6][6]);

/**
 * @brief Multiply a 6x6 matrix by a 6x3 matrix: C = A * B.
 * @param A 6x6 matrix.
 * @param B 6x3 matrix.
 * @param C Output 6x3 matrix.
 */
void mul6_6_3(const float A[6][6], const float B[6][3], float C[6][3]);

/**
 * @brief Multiply a 3x6 matrix by a 6x6 matrix: C = A * B.
 * @param A 3x6 matrix.
 * @param B 6x6 matrix.
 * @param C Output 3x6 matrix.
 */
void mul3_6_6(const float A[3][6], const float B[6][6], float C[3][6]);

/**
 * @brief Multiply a 3x6 matrix by a 6x3 matrix: C = A * B.
 * @param A 3x6 matrix.
 * @param B 6x3 matrix.
 * @param C Output 3x3 matrix.
 */
void mul3_6_3(const float A[3][6], const float B[6][3], float C[3][3]);

/**
 * @brief Multiply two 3x3 matrices: C = A * B.
 * @param A First 3x3 matrix.
 * @param B Second 3x3 matrix.
 * @param C Output 3x3 matrix.
 */
void mul3(const float A[3][3], const float B[3][3], float C[3][3]);

/**
 * @brief Transpose a 6x6 matrix.
 * @param A Input 6x6 matrix.
 * @param At Output 6x6 transposed matrix.
 */
void transpose6(const float A[6][6], float At[6][6]);

/**
 * @brief Transpose a 6x3 matrix to a 3x6 matrix.
 * @param A Input 6x3 matrix.
 * @param At Output 3x6 transposed matrix.
 */
void transpose6_3(const float A[6][3], float At[3][6]);

/**
 * @brief Transpose a 3x6 matrix to a 6x3 matrix.
 * @param A Input 3x6 matrix.
 * @param At Output 6x3 transposed matrix.
 */
void transpose3_6(const float A[3][6], float At[6][3]);

/**
 * @brief Add two 6x6 matrices: out = A + B.
 * @param A First 6x6 matrix.
 * @param B Second 6x6 matrix.
 * @param out Output 6x6 matrix.
 */
void add6(float A[6][6], const float B[6][6], float out[6][6]);

/**
 * @brief Add two 3x3 matrices: out = A + B.
 * @param A First 3x3 matrix.
 * @param B Second 3x3 matrix.
 * @param out Output 3x3 matrix.
 */
void add3(float A[3][3], const float B[3][3], float out[3][3]);

/**
 * @brief Invert a 3x3 matrix.
 * @param A Input 3x3 matrix.
 * @param Ainv Output 3x3 inverse matrix.
 * @return 0 on success, -1 if the matrix is singular.
 */
int inv3(const float A[3][3], float Ainv[3][3]);

#endif // SLAM_MAT_H
