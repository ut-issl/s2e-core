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
 * @brief Test for PartialDerivative calculation case 1
 */
TEST(GravityPotential, PartialDerivative1) {
  const size_t degree = 10;

  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients

  // Unit coefficients
  c_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));
  s_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));

  // Initialize GravityPotential
  GravityPotential gravity_potential_(degree, c_, s_, 1.0, 1.0);

  // Calculation check
  libra::Vector<3> position_xcxf_m;
  libra::Matrix<3, 3> partial_derivative_xcxf_s2;
  const double accuracy = 1.0e-3;

  // Calc Partial Derivative
  position_xcxf_m[0] = 1.0;
  position_xcxf_m[1] = 1.0;
  position_xcxf_m[2] = 1.0;
  partial_derivative_xcxf_s2 = gravity_potential_.CalcPartialDerivative_xcxf_s2(position_xcxf_m);

  // Calc Acceleration and numerical partial derivatives
  double d_r = 1e-9;
  libra::Matrix<3, 3> numerical_partial_derivative_xcxf_s2;
  for (size_t i = 0; i < 3; i++) {
    libra::Vector<3> position_1_xcxf_m = position_xcxf_m;
    libra::Vector<3> position_2_xcxf_m = position_xcxf_m;
    position_1_xcxf_m[i] = position_xcxf_m[i] - d_r / 2.0;
    position_2_xcxf_m[i] = position_xcxf_m[i] + d_r / 2.0;
    libra::Vector<3> acceleration_1_xcxf_m_s2 = gravity_potential_.CalcAcceleration_xcxf_m_s2(position_1_xcxf_m);
    libra::Vector<3> acceleration_2_xcxf_m_s2 = gravity_potential_.CalcAcceleration_xcxf_m_s2(position_2_xcxf_m);
    libra::Vector<3> diff_acceleration_xcxf_m_s2 = acceleration_2_xcxf_m_s2 - acceleration_1_xcxf_m_s2;
    for (size_t j = 0; j < 3; j++) {
      numerical_partial_derivative_xcxf_s2[i][j] = diff_acceleration_xcxf_m_s2[j] / d_r;
    }
  }

  // Compare numerical and analytical calculation
  libra::Matrix<3, 3> diff;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      EXPECT_NEAR(numerical_partial_derivative_xcxf_s2[i][j], partial_derivative_xcxf_s2[i][j], accuracy);
      diff[i][j] = numerical_partial_derivative_xcxf_s2[i][j] - partial_derivative_xcxf_s2[i][j];
    }
  }
}

/**
 * @brief Test for PartialDerivative calculation case2
 */
TEST(GravityPotential, PartialDerivative2) {
  const size_t degree = 10;

  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients

  // Unit coefficients
  c_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));
  s_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));

  // Initialize GravityPotential
  GravityPotential gravity_potential_(degree, c_, s_, 1.0, 1.0);

  // Calculation check
  libra::Vector<3> position_xcxf_m;
  libra::Matrix<3, 3> partial_derivative_xcxf_s2;
  const double accuracy = 1.0e-3;

  // Calc Partial Derivative
  position_xcxf_m[0] = 1.0;
  position_xcxf_m[1] = 0.0;
  position_xcxf_m[2] = 1.0;
  partial_derivative_xcxf_s2 = gravity_potential_.CalcPartialDerivative_xcxf_s2(position_xcxf_m);

  // Calc Acceleration and numerical partial derivatives
  double d_r = 1e-9;
  libra::Matrix<3, 3> numerical_partial_derivative_xcxf_s2;
  for (size_t i = 0; i < 3; i++) {
    libra::Vector<3> position_1_xcxf_m = position_xcxf_m;
    libra::Vector<3> position_2_xcxf_m = position_xcxf_m;
    position_1_xcxf_m[i] = position_xcxf_m[i] - d_r / 2.0;
    position_2_xcxf_m[i] = position_xcxf_m[i] + d_r / 2.0;
    libra::Vector<3> acceleration_1_xcxf_m_s2 = gravity_potential_.CalcAcceleration_xcxf_m_s2(position_1_xcxf_m);
    libra::Vector<3> acceleration_2_xcxf_m_s2 = gravity_potential_.CalcAcceleration_xcxf_m_s2(position_2_xcxf_m);
    libra::Vector<3> diff_acceleration_xcxf_m_s2 = acceleration_2_xcxf_m_s2 - acceleration_1_xcxf_m_s2;
    for (size_t j = 0; j < 3; j++) {
      numerical_partial_derivative_xcxf_s2[i][j] = diff_acceleration_xcxf_m_s2[j] / d_r;
    }
  }

  // Compare numerical and analytical calculation
  libra::Matrix<3, 3> diff;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      EXPECT_NEAR(numerical_partial_derivative_xcxf_s2[i][j], partial_derivative_xcxf_s2[i][j], accuracy);
      diff[i][j] = numerical_partial_derivative_xcxf_s2[i][j] - partial_derivative_xcxf_s2[i][j];
    }
  }
}
