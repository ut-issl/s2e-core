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
  math::Vector<4> v(0.5);
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
  math::Vector<3> axis;
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
  math::Vector<3> axis;
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
  math::Vector<3> axis;
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
  math::Vector<3> axis;
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
  math::Vector<3> before;
  before[0] = 0.0;
  before[1] = 0.0;
  before[2] = 2.0;  // To check normalization
  math::Vector<3> after;
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
  math::Vector<3> before;
  before[0] = 0.0;
  before[1] = 0.0;
  before[2] = 1.0;
  math::Vector<3> after;
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
  math::Vector<3> before;
  before[0] = 0.0;
  before[1] = 0.0;
  before[2] = 1.0;
  math::Vector<3> after;
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
  math::Vector<3> before;
  before[0] = 1.0;
  before[1] = 0.0;
  before[2] = 0.0;
  math::Vector<3> after;
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

/**
 * @brief Test for ConvertToDcm Y rotation
 */
TEST(Quaternion, ConvertToDcmY) {
  libra::Quaternion q(0.0, 1.0, 0.0, 1.0);
  q.Normalize();

  math::Matrix<3, 3> dcm = q.ConvertToDcm();

  // Check nondestructive function
  EXPECT_DOUBLE_EQ(0.0, q[0]);
  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), q[1]);
  EXPECT_DOUBLE_EQ(0.0, q[2]);
  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), q[3]);

  // Check nondestructive function
  const double accuracy = 1.0e-7;
  EXPECT_NEAR(0.0, dcm[0][0], accuracy);
  EXPECT_NEAR(0.0, dcm[0][1], accuracy);
  EXPECT_NEAR(-1.0, dcm[0][2], accuracy);
  EXPECT_NEAR(0.0, dcm[1][0], accuracy);
  EXPECT_NEAR(1.0, dcm[1][1], accuracy);
  EXPECT_NEAR(0.0, dcm[1][2], accuracy);
  EXPECT_NEAR(1.0, dcm[2][0], accuracy);
  EXPECT_NEAR(0.0, dcm[2][1], accuracy);
  EXPECT_NEAR(0.0, dcm[2][2], accuracy);

  // Inverse Conversion
  libra::Quaternion q_from_dcm = libra::Quaternion::ConvertFromDcm(dcm);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_NEAR(q[i], q_from_dcm[i], accuracy);
  }
}

/**
 * @brief Test for ConvertToDcm
 */
TEST(Quaternion, ConvertToDcm) {
  libra::Quaternion q(0.5, 0.3, 0.1, 1.0);
  q.Normalize();

  math::Matrix<3, 3> dcm = q.ConvertToDcm();

  // Check nondestructive function
  const double accuracy = 1.0e-5;
  EXPECT_NEAR(0.8518519, dcm[0][0], accuracy);
  EXPECT_NEAR(0.3703704, dcm[0][1], accuracy);
  EXPECT_NEAR(-0.3703704, dcm[0][2], accuracy);
  EXPECT_NEAR(0.0740741, dcm[1][0], accuracy);
  EXPECT_NEAR(0.6148148, dcm[1][1], accuracy);
  EXPECT_NEAR(0.7851851, dcm[1][2], accuracy);
  EXPECT_NEAR(0.5185185, dcm[2][0], accuracy);
  EXPECT_NEAR(-0.696296, dcm[2][1], accuracy);
  EXPECT_NEAR(0.4962963, dcm[2][2], accuracy);

  // Inverse Conversion
  libra::Quaternion q_from_dcm = libra::Quaternion::ConvertFromDcm(dcm);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_NEAR(q[i], q_from_dcm[i], accuracy);
  }
}

/**
 * @brief Test for ConvertToEuler X rotation
 */
TEST(Quaternion, ConvertToEulerX) {
  libra::Quaternion q(1.0, 0.0, 0.0, 1.0);
  q.Normalize();

  math::Vector<3> euler = q.ConvertToEuler();

  // Check nondestructive function
  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), q[0]);
  EXPECT_DOUBLE_EQ(0.0, q[1]);
  EXPECT_DOUBLE_EQ(0.0, q[2]);
  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), q[3]);

  // Check nondestructive function
  const double accuracy = 1.0e-7;
  EXPECT_NEAR(90 * libra::deg_to_rad, euler[0], accuracy);
  EXPECT_NEAR(0.0, euler[1], accuracy);
  EXPECT_NEAR(0.0, euler[2], accuracy);

  // Inverse Conversion
  libra::Quaternion q_from_euler = libra::Quaternion::ConvertFromEuler(euler);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_NEAR(q[i], q_from_euler[i], accuracy);
  }
}

/**
 * @brief Test for ConvertToEuler
 */
TEST(Quaternion, ConvertToEuler) {
  libra::Quaternion q(0.5, 0.3, 0.1, 1.0);
  q.Normalize();

  math::Vector<3> euler = q.ConvertToEuler();

  // Check nondestructive function
  const double accuracy = 1.0e-7;
  EXPECT_NEAR(1.00712520, euler[0], accuracy);
  EXPECT_NEAR(0.37940772, euler[1], accuracy);
  EXPECT_NEAR(0.41012734, euler[2], accuracy);

  // Inverse Conversion
  libra::Quaternion q_from_euler = libra::Quaternion::ConvertFromEuler(euler);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_NEAR(q[i], q_from_euler[i], accuracy);
  }
}

/**
 * @brief Test for FrameConversion Z rotation
 */
TEST(Quaternion, FrameConversionZ) {
  libra::Quaternion q(0.0, 0.0, 1.0, 1.0);
  q.Normalize();

  math::Vector<3> v;
  v[0] = 1.0;
  v[1] = 0.0;
  v[2] = 0.0;

  math::Vector<3> v_frame_conv = q.FrameConversion(v);

  const double accuracy = 1.0e-7;
  EXPECT_NEAR(0.0, v_frame_conv[0], accuracy);
  EXPECT_NEAR(-1.0, v_frame_conv[1], accuracy);
  EXPECT_NEAR(0.0, v_frame_conv[2], accuracy);

  math::Vector<3> v_frame_conv_inv = q.InverseFrameConversion(v_frame_conv);

  for (size_t i = 0; i < 3; i++) {
    EXPECT_NEAR(v[i], v_frame_conv_inv[i], accuracy);
  }
}

/**
 * @brief Test for FrameConversion
 */
TEST(Quaternion, FrameConversion) {
  libra::Quaternion q(0.5, 0.3, 0.1, 1.0);
  q.Normalize();
  math::Vector<3> v;
  v[0] = 1.0;
  v[1] = 0.0;
  v[2] = 0.0;

  math::Vector<3> v_frame_conv = q.FrameConversion(v);
  math::Vector<3> v_frame_conv_inv = q.InverseFrameConversion(v_frame_conv);

  const double accuracy = 1.0e-7;
  for (size_t i = 0; i < 3; i++) {
    EXPECT_NEAR(v[i], v_frame_conv_inv[i], accuracy);
  }
}

/**
 * @brief Test for ConvertToVector
 */
TEST(Quaternion, ConvertToVector) {
  libra::Quaternion q(0.5, 0.3, 0.1, 1.0);

  math::Vector<4> v = q.ConvertToVector();

  for (size_t i = 0; i < 4; i++) {
    EXPECT_DOUBLE_EQ(q[i], v[i]);
  }
}

/**
 * @brief Test for operator+
 */
TEST(Quaternion, OperatorPlus) {
  libra::Quaternion q1(0.5, 0.3, 0.1, 1.0);
  libra::Quaternion q2(-0.3, 0.1, -1.0, 0.4);

  libra::Quaternion result = q1 + q2;

  EXPECT_DOUBLE_EQ(0.2, result[0]);
  EXPECT_DOUBLE_EQ(0.4, result[1]);
  EXPECT_DOUBLE_EQ(-0.9, result[2]);
  EXPECT_DOUBLE_EQ(1.4, result[3]);
}

/**
 * @brief Test for operator-
 */
TEST(Quaternion, OperatorMinus) {
  libra::Quaternion q1(0.5, 0.3, 0.1, 1.0);
  libra::Quaternion q2(-0.3, 0.1, -1.0, 0.4);

  libra::Quaternion result = q1 - q2;

  EXPECT_DOUBLE_EQ(0.8, result[0]);
  EXPECT_DOUBLE_EQ(0.2, result[1]);
  EXPECT_DOUBLE_EQ(1.1, result[2]);
  EXPECT_DOUBLE_EQ(0.6, result[3]);
}

/**
 * @brief Test for operator* quaternion
 */
TEST(Quaternion, OperatorQuaternionMultiply) {
  libra::Quaternion q1(0.289271, -0.576012, -0.420972, 0.638212);
  libra::Quaternion q2(-0.0821846, 0.501761, 0.721995, -0.469259);

  libra::Quaternion result = q1 * q2;

  const double accuracy = 1.0e-7;
  EXPECT_NEAR(-0.3928446703722, result[0], accuracy);
  EXPECT_NEAR(0.4162739062262, result[1], accuracy);
  EXPECT_NEAR(0.7561363631038, result[2], accuracy);
  EXPECT_NEAR(0.3172469327906, result[3], accuracy);
}

/**
 * @brief Test for operator* scalar
 */
TEST(Quaternion, OperatorScalarMultiply) {
  libra::Quaternion q(0.289271, -0.576012, -0.420972, 0.638212);
  double scalar = 2.3;

  libra::Quaternion result = scalar * q;

  const double accuracy = 1.0e-7;
  EXPECT_NEAR(q[0] * 2.3, result[0], accuracy);
  EXPECT_NEAR(q[1] * 2.3, result[1], accuracy);
  EXPECT_NEAR(q[2] * 2.3, result[2], accuracy);
  EXPECT_NEAR(q[3] * 2.3, result[3], accuracy);
}
