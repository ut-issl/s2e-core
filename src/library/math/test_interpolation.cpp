/**
 * @file test_interpolation.cpp
 * @brief Test codes for Interpolation class with GoogleTest
 */
#include <gtest/gtest.h>

#include "constants.hpp"
#include "interpolation.hpp"

/**
 * @brief Test for linear function with polynomial interpolation
 */
TEST(Interpolation, PolynomialLinearFunction) {
  std::vector<double> x{0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> y{0.0, 2.0, 4.0, 6.0, 8.0};
  libra::Interpolation interpolation(x, y);

  double xx = 0.4;
  EXPECT_DOUBLE_EQ(2.0 * xx, interpolation.CalcPolynomial(xx));
  xx = 2.6;
  EXPECT_DOUBLE_EQ(2.0 * xx, interpolation.CalcPolynomial(xx));
  xx = 3.6;
  EXPECT_DOUBLE_EQ(2.0 * xx, interpolation.CalcPolynomial(xx));
}

/**
 * @brief Test for quadratic function with polynomial interpolation
 */
TEST(Interpolation, PolynomialQuadraticFunction) {
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

/**
 * @brief Test for sin function with trigonometric interpolation
 */
TEST(Interpolation, TrigonometricSinFunction) {
  std::vector<double> x{0.0, libra::pi_2, libra::pi, libra::pi * 3.0 / 2.0, libra::tau};
  std::vector<double> y;
  for (size_t i = 0; i < x.size(); i++) {
    y.push_back(sin(x[i]));
  }
  libra::Interpolation interpolation(x, y);

  double xx = 0.4 * libra::pi;
  EXPECT_DOUBLE_EQ(sin(xx), interpolation.CalcTrigonometric(xx));
  xx = 1.4 * libra::pi;
  EXPECT_DOUBLE_EQ(sin(xx), interpolation.CalcTrigonometric(xx));
}

/**
 * @brief Test for cos function with trigonometric interpolation
 */
TEST(Interpolation, TrigonometricCosFunction) {
  std::vector<double> x{0.0, 0.3 * libra::pi_2, 0.6 * libra::pi_2, 0.9 * libra::pi_2, 1.2 * libra::pi_2};
  std::vector<double> y;
  for (size_t i = 0; i < x.size(); i++) {
    y.push_back(cos(x[i]));
  }
  libra::Interpolation interpolation(x, y);

  double xx = 0.1 * libra::pi_2;
  EXPECT_NEAR(cos(xx), interpolation.CalcTrigonometric(xx), 1e-6);
  xx = 0.8 * libra::pi_2;
  EXPECT_NEAR(cos(xx), interpolation.CalcTrigonometric(xx), 1e-6);
}
/**
 * @brief Test for cos function with trigonometric interpolation
 */
TEST(Interpolation, TrigonometricCosSinFunctionOddDegree) {
  std::vector<double> x{0.0, 0.3 * libra::pi_2, 0.6 * libra::pi_2, 0.9 * libra::pi_2, 1.2 * libra::pi_2, 1.5 * libra::pi_2};
  std::vector<double> y;
  for (size_t i = 0; i < x.size(); i++) {
    y.push_back(cos(x[i]) + sin(x[i]));
  }
  libra::Interpolation interpolation(x, y);

  double xx = 0.1 * libra::pi_2;
  EXPECT_NEAR(cos(xx) + sin(xx), interpolation.CalcTrigonometric(xx), 1e-6);
  xx = 0.8 * libra::pi_2;
  EXPECT_NEAR(cos(xx) + sin(xx), interpolation.CalcTrigonometric(xx), 1e-6);
}
