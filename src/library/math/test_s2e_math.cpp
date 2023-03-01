/**
 * @file test_quaternion.cpp
 * @brief Test codes for Quaternion class with GoogleTest
 */
#include <gtest/gtest.h>

#include "constants.hpp"
#include "s2e_math.hpp"

/**
 * @brief Test for constructor with number
 */
TEST(S2eMath, WrapTo2Pi) {
  const double accuracy = 1.0e-7;
  
  double input_angle_rad = 0.0;
  double wrapped_angle_rad = libra::WrapTo2Pi(input_angle_rad);
  EXPECT_NEAR(0.0, wrapped_angle_rad, accuracy);

  input_angle_rad = -1.0e-5;
  wrapped_angle_rad = libra::WrapTo2Pi(input_angle_rad);
  EXPECT_NEAR(libra::tau + input_angle_rad, wrapped_angle_rad, accuracy);

  input_angle_rad = libra::tau + 1.0e-5;
  wrapped_angle_rad = libra::WrapTo2Pi(input_angle_rad);
  EXPECT_NEAR(1.0e-5, wrapped_angle_rad, accuracy);
}
