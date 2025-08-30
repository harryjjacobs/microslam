#ifndef SLAM_MAT_H
#define SLAM_MAT_H

#include "slam/ekf.h"

/**
 * @brief Set all elements of a 3x3 matrix to zero.
 * @param A The 3x3 matrix to zero.
 */
void zero3(float A[9]);

/**
 * @brief Multiply two 3x3 matrices: C = A * B.
 * @param A First 3x3 matrix.
 * @param B Second 3x3 matrix.
 * @param C Output 3x3 matrix.
 */
void mul3(const float A[9], const float B[9], float C[9]);

/**
 * @brief Transpose a 3x3 matrix.
 * @param A Input 3x3 matrix.
 * @param At Output 3x3 transposed matrix.
 */
void transpose3(const float A[9], float At[9]);

/**
 * @brief Add two 3x3 matrices: out = A + B.
 * @param A First 3x3 matrix.
 * @param B Second 3x3 matrix.
 * @param out Output 3x3 matrix.
 */
void add3(const float A[9], const float B[9], float out[9]);

/**
 * @brief Invert a 3x3 matrix.
 * @param A Input 3x3 matrix.
 * @param Ainv Output 3x3 inverse matrix.
 * @return 0 on success, -1 if the matrix is singular.
 */
int inv3(const float A[9], float Ainv[9]);

/**
 * @brief Symmetrize a 3x3 matrix by averaging its off-diagonal elements.
 * @param A Input 3x3 matrix.
 */
void symmetrize3(float A[9]);

#endif // SLAM_MAT_H
