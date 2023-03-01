/**
 * @file test_quaternion.cpp
 * @brief Test codes for Quaternion class with GoogleTest
 */
#include <gtest/gtest.h>

#include "constants.hpp"
#include "matrix.hpp"

/**
 * @brief Test for constructor with number
 */
TEST(Matrix, ConstructorWithNumber) {
  const size_t R = 6;
  const size_t C = 3;
  double initialize_value = 2.0;
  libra::Matrix<R, C> m(initialize_value);

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      EXPECT_DOUBLE_EQ(initialize_value, m[r][c]);
      EXPECT_DOUBLE_EQ(initialize_value, m(r, c));
    }
  }
}
