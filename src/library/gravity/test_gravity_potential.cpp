/**
 * @file test_gravity_potential.cpp
 * @brief Test codes for Gravity Potential class with GoogleTest
 */
#include <gtest/gtest.h>

#include "gravity_potential.hpp"

/**
 * @brief Test for constructor with number
 */
TEST(Gravity UnitCoefficients) {
  const size_t degree = 1;

  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients

  // Unit coefficients
  c_.assign(degree_ + 1, std::vector<double>(degree_ + 1, 1.0));
  s_.assign(degree_ + 1, std::vector<double>(degree_ + 1, 1.0));

  // Initialize GravityPotential
  GravityPotential gravity_potential_(degree, c_, s_, 1, 1);

  // Calc Acceleration
  libra::Vector<3> position_xcxf_m;
  position_xcxf_m[0] = 1.0;
  position_xcxf_m[1] = 0.0;
  position_xcxf_m[2] = 0.0;
  libra::Vector<3> acceleration_xcxf_m_s2 = CalcAcceleration_xcxf_m_s2(position_xcxf_m);

  // Check
  EXPECT_DOUBLE_EQ(-1, acceleration_xcxf_m_s2[0]);
  EXPECT_DOUBLE_EQ(0.0, acceleration_xcxf_m_s2[1]);
  EXPECT_DOUBLE_EQ(0.0, acceleration_xcxf_m_s2[2]);
}
