/**
 * @file test_quaternion.cpp
 * @brief Test codes for Quaternion class with GoogleTest
 */
#include <gtest/gtest.h>

#include "quaternion.hpp"

TEST(Quaternion, ConstructorFourNumber) {
  libra::Quaternion q(0.5, 0.5, 0.5, 0.5);

  EXPECT_DOUBLE_EQ(0.5, q[0]);
  EXPECT_DOUBLE_EQ(0.5, q[1]);
  EXPECT_DOUBLE_EQ(0.5, q[2]);
  EXPECT_DOUBLE_EQ(0.5, q[3]);
}

TEST(Quaternion, ConstructorVector) {
  libra::Vector<4> v(0.5);
  libra::Quaternion q(v);

  EXPECT_DOUBLE_EQ(0.5, q[0]);
  EXPECT_DOUBLE_EQ(0.5, q[1]);
  EXPECT_DOUBLE_EQ(0.5, q[2]);
  EXPECT_DOUBLE_EQ(0.5, q[3]);
}

TEST(Quaternion, ConstructorAxisRot) {
  libra::Vector<3> axis;
  axis[0] = 1.0;
  axis[1] = 0.0;
  axis[2] = 0.0;
  double theta_rad = 1.5708;
  libra::Quaternion q(axis, theta_rad);

  EXPECT_NEAR(1 / sqrt(2), q[0], 1e-5);
  EXPECT_NEAR(0.0, q[1], 1e-5);
  EXPECT_NEAR(0.0, q[2], 1e-5);
  EXPECT_NEAR(1 / sqrt(2), q[3], 1e-5);
}
