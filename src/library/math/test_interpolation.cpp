/**
 * @file test_interpolation.cpp
 * @brief Test codes for Interpolation class with GoogleTest
 */
#include <gtest/gtest.h>

#include "interpolation.hpp"

/**
 * @brief Test for linear function
 */
TEST(Interpolation, LinearFunction) {
  std::vector<double> x{0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> y{0.0, 2.0, 4.0, 6.0, 8.0};
  libra::Interpolation interpolation(x, y);

  double xx = 0.4;
  EXPECT_DOUBLE_EQ(2.0*xx, interpolation.CalcPolynomial(xx));
  xx = 2.6;
  EXPECT_DOUBLE_EQ(2.0*xx, interpolation.CalcPolynomial(xx));
  xx = 3.6;
  EXPECT_DOUBLE_EQ(2.0*xx, interpolation.CalcPolynomial(xx));
}

/**
 * @brief Test for quadratic function
 */
TEST(Interpolation, QuadraticFunction) {
  std::vector<double> x{0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> y{0.0, 1.0, 4.0, 9.0, 16.0};
  libra::Interpolation interpolation(x, y);

  double xx = 0.4;
  EXPECT_DOUBLE_EQ(pow(xx, 2.0), interpolation.CalcPolynomial(xx));
  xx = 2.4;
  EXPECT_DOUBLE_EQ(pow(xx, 2.0), interpolation.CalcPolynomial(xx));
  xx = 3.8;
  EXPECT_DOUBLE_EQ(pow(xx, 2.0), interpolation.CalcPolynomial(xx));
}
