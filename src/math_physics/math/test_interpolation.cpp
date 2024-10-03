/**
 * @file test_interpolation.cpp
 * @brief Test codes for Interpolation class with GoogleTest
 */
#include <gtest/gtest.h>

#include <cmath>

#include "constants.hpp"
#include "interpolation.hpp"

/**
 * @brief Test for linear function with polynomial interpolation
 */
TEST(Interpolation, PolynomialLinearFunction) {
  std::vector<double> x{0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> y{0.0, 2.0, 4.0, 6.0, 8.0};
  s2e::math::Interpolation interpolation(x, y);

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
  s2e::math::Interpolation interpolation(x, y);

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
  std::vector<double> x{0.0, s2e::math::pi_2, s2e::math::pi, s2e::math::pi * 3.0 / 2.0, s2e::math::tau};
  std::vector<double> y;
  for (size_t i = 0; i < x.size(); i++) {
    y.push_back(sin(x[i]));
  }
  s2e::math::Interpolation interpolation(x, y);

  double xx = 0.4 * s2e::math::pi;
  EXPECT_DOUBLE_EQ(sin(xx), interpolation.CalcTrigonometric(xx));
  xx = 1.4 * s2e::math::pi;
  EXPECT_DOUBLE_EQ(sin(xx), interpolation.CalcTrigonometric(xx));
}

/**
 * @brief Test for cos function with trigonometric interpolation
 */
TEST(Interpolation, TrigonometricCosFunction) {
  std::vector<double> x{0.0, 0.3 * s2e::math::pi_2, 0.6 * s2e::math::pi_2, 0.9 * s2e::math::pi_2, 1.2 * s2e::math::pi_2};
  std::vector<double> y;
  for (size_t i = 0; i < x.size(); i++) {
    y.push_back(cos(x[i]));
  }
  s2e::math::Interpolation interpolation(x, y);

  double xx = 0.1 * s2e::math::pi_2;
  EXPECT_NEAR(cos(xx), interpolation.CalcTrigonometric(xx), 1e-6);
  xx = 0.8 * s2e::math::pi_2;
  EXPECT_NEAR(cos(xx), interpolation.CalcTrigonometric(xx), 1e-6);
}

/**
 * @brief Test for cos function with trigonometric interpolation
 */
TEST(Interpolation, TrigonometricCosSinFunctionOddDegree) {
  std::vector<double> x{0.0, 0.3 * s2e::math::pi_2, 0.6 * s2e::math::pi_2, 0.9 * s2e::math::pi_2, 1.2 * s2e::math::pi_2, 1.5 * s2e::math::pi_2};
  std::vector<double> y;
  for (size_t i = 0; i < x.size(); i++) {
    y.push_back(cos(x[i]) + sin(x[i]));
  }
  s2e::math::Interpolation interpolation(x, y);

  double xx = 0.1 * s2e::math::pi_2;
  EXPECT_NEAR(cos(xx) + sin(xx), interpolation.CalcTrigonometric(xx), 1e-6);
  xx = 0.8 * s2e::math::pi_2;
  EXPECT_NEAR(cos(xx) + sin(xx), interpolation.CalcTrigonometric(xx), 1e-6);
}

/**
 * @brief Test for PushAndPop function
 */
TEST(Interpolation, PushAndPop) {
  std::vector<double> x{0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> y{0.0, 2.0, 4.0, 6.0, 8.0};
  s2e::math::Interpolation interpolation(x, y);

  EXPECT_EQ(x.size(), interpolation.GetDegree());
  for (size_t i = 0; i < x.size(); i++) {
    EXPECT_DOUBLE_EQ(x[i], interpolation.GetIndependentVariables()[i]);
    EXPECT_DOUBLE_EQ(y[i], interpolation.GetDependentVariables()[i]);
  }

  bool ret = interpolation.PushAndPopData(5.0, 10.0);
  EXPECT_TRUE(ret);
  EXPECT_DOUBLE_EQ(x[1], interpolation.GetIndependentVariables()[0]);
  EXPECT_DOUBLE_EQ(y[1], interpolation.GetDependentVariables()[0]);
  EXPECT_DOUBLE_EQ(5.0, interpolation.GetIndependentVariables()[4]);
  EXPECT_DOUBLE_EQ(10.0, interpolation.GetDependentVariables()[4]);

  // False test
  ret = interpolation.PushAndPopData(1.0, 10.0);
  EXPECT_FALSE(ret);
}
