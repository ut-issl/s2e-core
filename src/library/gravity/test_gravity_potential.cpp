/**
 * @file test_gravity_potential.cpp
 * @brief Test codes for Gravity Potential class with GoogleTest
 */
#include <gtest/gtest.h>

#include "gravity_potential.hpp"

/**
 * @brief Test for Acceleration calculation
 */
TEST(GravityPotential, Acceleration) {
  const size_t degree = 10;

  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients

  // Unit coefficients
  c_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));
  s_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));

  // Initialize GravityPotential
  GravityPotential gravity_potential_(degree, c_, s_, 1.0, 1.0);

  // Acceleration Calculation check
  libra::Vector<3> position_xcxf_m;
  libra::Vector<3> acceleration_xcxf_m_s2;
  const double accuracy = 1.0e-3;

  // Calc Acceleration
  position_xcxf_m[0] = 1.0;
  position_xcxf_m[1] = 0.0;
  position_xcxf_m[2] = 0.0;
  acceleration_xcxf_m_s2 = gravity_potential_.CalcAcceleration_xcxf_m_s2(position_xcxf_m);
  // Check
  EXPECT_NEAR(-100.0252, acceleration_xcxf_m_s2[0], accuracy);
  EXPECT_NEAR(93.3516, acceleration_xcxf_m_s2[1], accuracy);
  EXPECT_NEAR(41.0375, acceleration_xcxf_m_s2[2], accuracy);

  // Calc Acceleration
  position_xcxf_m[0] = 1.0;
  position_xcxf_m[1] = 1.0;
  position_xcxf_m[2] = 1.0;
  acceleration_xcxf_m_s2 = gravity_potential_.CalcAcceleration_xcxf_m_s2(position_xcxf_m);
  // Check
  EXPECT_NEAR(-0.19228, acceleration_xcxf_m_s2[0], accuracy);
  EXPECT_NEAR(-2.46144, acceleration_xcxf_m_s2[1], accuracy);
  EXPECT_NEAR(0.242614, acceleration_xcxf_m_s2[2], accuracy);
}

/**
 * @brief Test for PartialDerivative calculation
 */
TEST(GravityPotential, PartialDerivative) {
  const size_t degree = 10;

  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients

  // Unit coefficients
  c_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));
  s_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));

  // Initialize GravityPotential
  GravityPotential gravity_potential_(degree, c_, s_, 1.0, 1.0);

  // Acceleration Calculation check
  libra::Vector<3> position_xcxf_m;
  libra::Matrix<3, 3> partial_derivative_xcxf_s2;
  const double accuracy = 1.0e-3;

  // Calc Acceleration
  position_xcxf_m[0] = 1.0;
  position_xcxf_m[1] = 0.0;
  position_xcxf_m[2] = 0.0;
  partial_derivative_xcxf_s2 = gravity_potential_.CalcPartialDerivative_xcxf_s2(position_xcxf_m);
  // Check
  EXPECT_NEAR(876.35464, partial_derivative_xcxf_s2[0][0], accuracy);
  EXPECT_NEAR(-855.44816, partial_derivative_xcxf_s2[0][1], accuracy);
  EXPECT_NEAR(-3010.41183, partial_derivative_xcxf_s2[0][2], accuracy);
  EXPECT_NEAR(partial_derivative_xcxf_s2[0][1], partial_derivative_xcxf_s2[1][0], accuracy);
  EXPECT_NEAR(-784.31820, partial_derivative_xcxf_s2[1][1], accuracy);
  EXPECT_NEAR(2895.07489, partial_derivative_xcxf_s2[1][2], accuracy);
  EXPECT_NEAR(partial_derivative_xcxf_s2[0][2], partial_derivative_xcxf_s2[2][0], accuracy);
  EXPECT_NEAR(partial_derivative_xcxf_s2[1][2], partial_derivative_xcxf_s2[2][1], accuracy);
  EXPECT_NEAR(-90.04199, partial_derivative_xcxf_s2[2][2], accuracy);
}
