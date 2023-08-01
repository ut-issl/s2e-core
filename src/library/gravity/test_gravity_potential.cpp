/**
 * @file test_gravity_potential.cpp
 * @brief Test codes for Gravity Potential class with GoogleTest
 */
#include <gtest/gtest.h>

#include "gravity_potential.hpp"

/**
 * @brief Test for constructor with number
 */
TEST(GravityPotential, UnitCoefficients) {
  const size_t degree = 10;

  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients

  // Unit coefficients
  c_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));
  s_.assign(degree + 1, std::vector<double>(degree + 1, 1.0));

  // Initialize GravityPotential
  GravityPotential gravity_potential_(degree, c_, s_, 1, 1);

  // Calc Acceleration
  libra::Vector<3> position_xcxf_m;
  position_xcxf_m[0] = 1.0;
  position_xcxf_m[1] = 0.0;
  position_xcxf_m[2] = 0.0;
  libra::Vector<3> acceleration_xcxf_m_s2 = gravity_potential_.CalcAcceleration_xcxf_m_s2(position_xcxf_m);

  // Check
  const double accuracy = 1.0e-3;
  EXPECT_NEAR(-100.0252, acceleration_xcxf_m_s2[0], accuracy);
  EXPECT_NEAR(93.3516, acceleration_xcxf_m_s2[1], accuracy);
  EXPECT_NEAR(41.0375, acceleration_xcxf_m_s2[2], accuracy);
}
