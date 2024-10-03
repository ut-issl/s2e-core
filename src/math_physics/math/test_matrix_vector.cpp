/**
 * @file test_quaternion.cpp
 * @brief Test codes for Quaternion class with GoogleTest
 */
#include <gtest/gtest.h>

#include "constants.hpp"
#include "matrix_vector.hpp"

/**
 * @brief Test for Matrix * Vector
 */
TEST(MatrixVector, MultiplyMatrixVector) {
  const size_t R = 3;
  const size_t C = 2;

  s2e::math::Matrix<R, C> m;
  s2e::math::Vector<C> v;

  m[0][0] = 1.0;
  m[0][1] = 2.0;
  m[1][0] = -1.0;
  m[1][1] = 0.0;
  m[2][0] = 3.0;
  m[2][1] = 1.0;

  v[0] = 7.0;
  v[1] = 1.0;

  s2e::math::Vector<R> result = m * v;

  EXPECT_DOUBLE_EQ(9.0, result[0]);
  EXPECT_DOUBLE_EQ(-7.0, result[1]);
  EXPECT_DOUBLE_EQ(22.0, result[2]);
}

/**
 * @brief Test for CalcInverseMatrix
 */
TEST(MatrixVector, CalcInverseMatrix) {
  const size_t N = 3;

  s2e::math::Matrix<N, N> m;

  m[0][0] = 1.0;
  m[0][1] = 1.0;
  m[0][2] = -1.0;
  m[1][0] = -2.0;
  m[1][1] = -1.0;
  m[1][2] = 1.0;
  m[2][0] = -1.0;
  m[2][1] = -2.0;
  m[2][2] = 1.0;

  s2e::math::Matrix<N, N> inverse = s2e::math::CalcInverseMatrix(m);

  EXPECT_NEAR(-1.0, inverse[0][0], 1e-10);
  EXPECT_NEAR(-1.0, inverse[0][1], 1e-10);
  EXPECT_NEAR(0.0, inverse[0][2], 1e-10);
  EXPECT_NEAR(-1.0, inverse[1][0], 1e-10);
  EXPECT_NEAR(0.0, inverse[1][1], 1e-10);
  EXPECT_NEAR(-1.0, inverse[1][2], 1e-10);
  EXPECT_NEAR(-3.0, inverse[2][0], 1e-10);
  EXPECT_NEAR(-1.0, inverse[2][1], 1e-10);
  EXPECT_NEAR(-1.0, inverse[2][2], 1e-10);
}
