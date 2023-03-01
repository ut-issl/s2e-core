/**
 * @file test_quaternion.cpp
 * @brief Test codes for Quaternion class with GoogleTest
 */
#include <gtest/gtest.h>

#include "constants.hpp"
#include "quaternion.hpp"

/**
 * @brief Test for constructor from four numbers
 */
TEST(Quaternion, ConstructorFourNumber) {
  libra::Quaternion q(0.5, 0.5, 0.5, 0.5);

  EXPECT_DOUBLE_EQ(0.5, q[0]);
  EXPECT_DOUBLE_EQ(0.5, q[1]);
  EXPECT_DOUBLE_EQ(0.5, q[2]);
  EXPECT_DOUBLE_EQ(0.5, q[3]);
}

/**
 * @brief Test for constructor from Vector
 */
TEST(Quaternion, ConstructorVector) {
  libra::Vector<4> v(0.5);
  libra::Quaternion q(v);

  EXPECT_DOUBLE_EQ(0.5, q[0]);
  EXPECT_DOUBLE_EQ(0.5, q[1]);
  EXPECT_DOUBLE_EQ(0.5, q[2]);
  EXPECT_DOUBLE_EQ(0.5, q[3]);
}

/**
 * @brief Test for constructor from axis and rotation angle X rotation
 */
TEST(Quaternion, ConstructorAxisAndAngleX) {
  libra::Vector<3> axis;
  axis[0] = 1.0;
  axis[1] = 0.0;
  axis[2] = 0.0;
  double theta_rad = 90 * libra::deg_to_rad;
  libra::Quaternion q(axis, theta_rad);

  EXPECT_NEAR(axis[0] * sin(theta_rad / 2.0), q[0], 1e-5);
  EXPECT_NEAR(axis[1] * sin(theta_rad / 2.0), q[1], 1e-5);
  EXPECT_NEAR(axis[2] * sin(theta_rad / 2.0), q[2], 1e-5);
  EXPECT_NEAR(cos(theta_rad / 2.0), q[3], 1e-5);
}

/**
 * @brief Test for constructor from axis and rotation angle Y rotation
 */
TEST(Quaternion, ConstructorAxisAndAngleY) {
  libra::Vector<3> axis;
  axis[0] = 0.0;
  axis[1] = 1.0;
  axis[2] = 0.0;
  double theta_rad = 45 * libra::deg_to_rad;
  libra::Quaternion q(axis, theta_rad);

  EXPECT_NEAR(axis[0] * sin(theta_rad / 2.0), q[0], 1e-5);
  EXPECT_NEAR(axis[1] * sin(theta_rad / 2.0), q[1], 1e-5);
  EXPECT_NEAR(axis[2] * sin(theta_rad / 2.0), q[2], 1e-5);
  EXPECT_NEAR(cos(theta_rad / 2.0), q[3], 1e-5);
}

/**
 * @brief Test for constructor from axis and rotation angle Z rotation
 */
TEST(Quaternion, ConstructorAxisAndAngleZ) {
  libra::Vector<3> axis;
  axis[0] = 0.0;
  axis[1] = 0.0;
  axis[2] = 1.0;
  double theta_rad = -60 * libra::deg_to_rad;
  libra::Quaternion q(axis, theta_rad);

  EXPECT_NEAR(axis[0] * sin(theta_rad / 2.0), q[0], 1e-5);
  EXPECT_NEAR(axis[1] * sin(theta_rad / 2.0), q[1], 1e-5);
  EXPECT_NEAR(axis[2] * sin(theta_rad / 2.0), q[2], 1e-5);
  EXPECT_NEAR(cos(theta_rad / 2.0), q[3], 1e-5);
}

/**
 * @brief Test for constructor from axis and rotation angle All axes rotation
 */
TEST(Quaternion, ConstructorAxisAndAngleAll) {
  libra::Vector<3> axis;
  axis[0] = 1.0;
  axis[1] = 1.0;
  axis[2] = 1.0;
  double theta_rad = 180 * libra::deg_to_rad;
  libra::Quaternion q(axis, theta_rad);

  EXPECT_NEAR(axis[0] * sin(theta_rad / 2.0), q[0], 1e-5);
  EXPECT_NEAR(axis[1] * sin(theta_rad / 2.0), q[1], 1e-5);
  EXPECT_NEAR(axis[2] * sin(theta_rad / 2.0), q[2], 1e-5);
  EXPECT_NEAR(cos(theta_rad / 2.0), q[3], 1e-5);
}

/**
 * @brief Test for constructor from two vectors: No rotation
 */
TEST(Quaternion, ConstructorTwoVectorsNoRotation) {
  libra::Vector<3> before;
  before[0] = 0.0;
  before[1] = 0.0;
  before[2] = 2.0;  // To check normalization
  libra::Vector<3> after;
  after[0] = 0.0;
  after[1] = 0.0;
  after[2] = 1.0;

  libra::Quaternion q(before, after);

  EXPECT_NEAR(0.0, q[0], 1e-5);
  EXPECT_NEAR(0.0, q[1], 1e-5);
  EXPECT_NEAR(0.0, q[2], 1e-5);
  EXPECT_NEAR(1.0, q[3], 1e-5);
}

/**
 * @brief Test for constructor from two vectors: X rotation
 */
TEST(Quaternion, ConstructorTwoVectorsX) {
  libra::Vector<3> before;
  before[0] = 0.0;
  before[1] = 0.0;
  before[2] = 1.0;
  libra::Vector<3> after;
  after[0] = 0.0;
  after[1] = 1.0;
  after[2] = 0.0;

  libra::Quaternion q(before, after);

  double theta_rad = -90 * libra::deg_to_rad;
  EXPECT_NEAR(sin(theta_rad / 2.0), q[0], 1e-5);
  EXPECT_NEAR(0.0, q[1], 1e-5);
  EXPECT_NEAR(0.0, q[2], 1e-5);
  EXPECT_NEAR(cos(theta_rad / 2.0), q[3], 1e-5);
}

/**
 * @brief Test for constructor from two vectors: Y rotation
 */
TEST(Quaternion, ConstructorTwoVectorsY) {
  libra::Vector<3> before;
  before[0] = 0.0;
  before[1] = 0.0;
  before[2] = 1.0;
  libra::Vector<3> after;
  after[0] = 1.0;
  after[1] = 0.0;
  after[2] = 0.0;

  libra::Quaternion q(before, after);

  double theta_rad = 90 * libra::deg_to_rad;
  EXPECT_NEAR(0.0, q[0], 1e-5);
  EXPECT_NEAR(sin(theta_rad / 2.0), q[1], 1e-5);
  EXPECT_NEAR(0.0, q[2], 1e-5);
  EXPECT_NEAR(cos(theta_rad / 2.0), q[3], 1e-5);
}

/**
 * @brief Test for constructor from two vectors: Z rotation
 */
TEST(Quaternion, ConstructorTwoVectorsZ) {
  libra::Vector<3> before;
  before[0] = 1.0;
  before[1] = 0.0;
  before[2] = 0.0;
  libra::Vector<3> after;
  after[0] = 0.0;
  after[1] = 1.0;
  after[2] = 0.0;

  libra::Quaternion q(before, after);

  double theta_rad = 90 * libra::deg_to_rad;
  EXPECT_NEAR(0.0, q[0], 1e-5);
  EXPECT_NEAR(0.0, q[1], 1e-5);
  EXPECT_NEAR(sin(theta_rad / 2.0), q[2], 1e-5);
  EXPECT_NEAR(cos(theta_rad / 2.0), q[3], 1e-5);
}

/**
 * @brief Test for Normalize
 * @note TODO: Fix to nondestructive function
 */
TEST(Quaternion, Normalize) {
  libra::Quaternion q(1.0, 1.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(1.0, q[0]);
  EXPECT_DOUBLE_EQ(1.0, q[1]);
  EXPECT_DOUBLE_EQ(1.0, q[2]);
  EXPECT_DOUBLE_EQ(1.0, q[3]);

  q.Normalize();
  EXPECT_DOUBLE_EQ(0.5, q[0]);
  EXPECT_DOUBLE_EQ(0.5, q[1]);
  EXPECT_DOUBLE_EQ(0.5, q[2]);
  EXPECT_DOUBLE_EQ(0.5, q[3]);
}

/**
 * @brief Test for Conjugate
 */
TEST(Quaternion, Conjugate) {
  libra::Quaternion q(0.5, 0.5, 0.5, 0.5);

  libra::Quaternion q_conjugate = q.Conjugate();

  // Check nondestructive function
  EXPECT_DOUBLE_EQ(0.5, q[0]);
  EXPECT_DOUBLE_EQ(0.5, q[1]);
  EXPECT_DOUBLE_EQ(0.5, q[2]);
  EXPECT_DOUBLE_EQ(0.5, q[3]);

  EXPECT_DOUBLE_EQ(-0.5, q_conjugate[0]);
  EXPECT_DOUBLE_EQ(-0.5, q_conjugate[1]);
  EXPECT_DOUBLE_EQ(-0.5, q_conjugate[2]);
  EXPECT_DOUBLE_EQ(0.5, q_conjugate[3]);
}
