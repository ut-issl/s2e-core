/**
 * @file test_quaternion.cpp
 * @brief Test codes for Quaternion class with GoogleTest
 */
#include <gtest/gtest.h>

#include "constants.hpp"
#include "vector.hpp"

/**
 * @brief Test for constructor with number
 */
TEST(Vector, ConstructorWithNumber) {
  const size_t N = 6;
  double initialize_value = 2.0;
  s2e::math::Vector<N> v(initialize_value);

  for (size_t i = 0; i < N; i++) {
    EXPECT_DOUBLE_EQ(initialize_value, v[i]);
    EXPECT_DOUBLE_EQ(initialize_value, v(i));
  }
}

/**
 * @brief Test for GetLength
 */
TEST(Vector, GetLength) {
  const size_t N = 6;
  s2e::math::Vector<N> v;

  EXPECT_EQ(N, v.GetLength());
}

/**
 * @brief Test for operator+=
 */
TEST(Vector, OperatorPlusEqual) {
  const size_t N = 6;
  double initialize_value = 2.0;
  s2e::math::Vector<N> v(initialize_value);
  s2e::math::Vector<N> adding;

  for (size_t i = 0; i < N; i++) {
    adding[i] = double(i);
  }

  v += adding;

  for (size_t i = 0; i < N; i++) {
    // Check nondestructive
    EXPECT_DOUBLE_EQ(double(i), adding[i]);
    // Check added value
    EXPECT_DOUBLE_EQ(initialize_value + double(i), v[i]);
  }
}

/**
 * @brief Test for operator-=
 */
TEST(Vector, OperatorMinusEqual) {
  const size_t N = 6;
  double initialize_value = 2.0;
  s2e::math::Vector<N> v(initialize_value);
  s2e::math::Vector<N> subtracting;

  for (size_t i = 0; i < N; i++) {
    subtracting[i] = double(i);
  }

  v -= subtracting;

  for (size_t i = 0; i < N; i++) {
    // Check nondestructive
    EXPECT_DOUBLE_EQ(double(i), subtracting[i]);
    // Check subtracted value
    EXPECT_DOUBLE_EQ(initialize_value - double(i), v[i]);
  }
}

/**
 * @brief Test for operator*=
 */
TEST(Vector, OperatorMultiplyEqual) {
  const size_t N = 6;
  s2e::math::Vector<N> v;
  double multiplying = 2.0;

  for (size_t i = 0; i < N; i++) {
    v[i] = double(i);
  }

  v *= multiplying;

  // Check nondestructive
  EXPECT_DOUBLE_EQ(2.0, multiplying);
  for (size_t i = 0; i < N; i++) {
    // Check multiplied value
    EXPECT_DOUBLE_EQ(multiplying * double(i), v[i]);
  }
}

/**
 * @brief Test for operator/=
 */
TEST(Vector, OperatorDivideEqual) {
  const size_t N = 6;
  s2e::math::Vector<N> v;
  double dividing = 3.0;

  for (size_t i = 0; i < N; i++) {
    v[i] = double(i);
  }

  v /= dividing;

  // Check nondestructive
  EXPECT_DOUBLE_EQ(3.0, dividing);
  for (size_t i = 0; i < N; i++) {
    // Check divided value
    EXPECT_DOUBLE_EQ(double(i) / dividing, v[i]);
  }
}

/**
 * @brief Test for operator-
 */
TEST(Vector, OperatorNegative) {
  const size_t N = 6;
  s2e::math::Vector<N> v;

  for (size_t i = 0; i < N; i++) {
    v[i] = double(i);
  }

  s2e::math::Vector<N> v_negative = -v;

  for (size_t i = 0; i < N; i++) {
    // Check nondestructive
    EXPECT_DOUBLE_EQ(double(i), v[i]);
    // Check negative value
    EXPECT_DOUBLE_EQ(double(i) * -1.0, v_negative[i]);
  }
}

/**
 * @brief Test for FillUp
 */
TEST(Vector, FillUp) {
  const size_t N = 6;
  s2e::math::Vector<N> v;

  for (size_t i = 0; i < N; i++) {
    v[i] = double(i);
  }

  for (size_t i = 0; i < N; i++) {
    EXPECT_DOUBLE_EQ(double(i), v[i]);
  }

  double fill_up_value = 0.1;
  v.FillUp(fill_up_value);

  for (size_t i = 0; i < N; i++) {
    EXPECT_DOUBLE_EQ(fill_up_value, v[i]);
  }
}

/**
 * @brief Test for operator+
 */
TEST(Vector, OperatorPlus) {
  const size_t N = 6;
  double initialize_value = 2.0;
  s2e::math::Vector<N> v(initialize_value);
  s2e::math::Vector<N> adding;

  for (size_t i = 0; i < N; i++) {
    adding[i] = double(i);
  }

  s2e::math::Vector<N> added = v + adding;

  for (size_t i = 0; i < N; i++) {
    // Check nondestructive
    EXPECT_DOUBLE_EQ(initialize_value, v[i]);
    EXPECT_DOUBLE_EQ(double(i), adding[i]);
    // Check added value
    EXPECT_DOUBLE_EQ(initialize_value + adding[i], added[i]);
  }
}

/**
 * @brief Test for operator-
 */
TEST(Vector, OperatorMinus) {
  const size_t N = 6;
  double initialize_value = 2.0;
  s2e::math::Vector<N> v(initialize_value);
  s2e::math::Vector<N> subtracting;

  for (size_t i = 0; i < N; i++) {
    subtracting[i] = double(i);
  }

  s2e::math::Vector<N> subtracted = v - subtracting;

  for (size_t i = 0; i < N; i++) {
    // Check nondestructive
    EXPECT_DOUBLE_EQ(initialize_value, v[i]);
    EXPECT_DOUBLE_EQ(double(i), subtracting[i]);
    // Check subtracted value
    EXPECT_DOUBLE_EQ(initialize_value - subtracting[i], subtracted[i]);
  }
}

/**
 * @brief Test for InnerProduct
 */
TEST(Vector, InnerProduct) {
  const size_t N = 3;
  s2e::math::Vector<N> a;
  s2e::math::Vector<N> b;

  for (size_t i = 0; i < N; i++) {
    a[i] = double(i + 1);
    b[i] = 1.0 / double(i + 1);
  }

  double result = InnerProduct(a, b);
  EXPECT_DOUBLE_EQ(double(N), result);
}

/**
 * @brief Test for InnerProduct result zero
 */
TEST(Vector, InnerProductZero) {
  const size_t N = 3;
  s2e::math::Vector<N> a;
  s2e::math::Vector<N> b;

  a[0] = 1.0;
  a[1] = 0.0;
  a[2] = 0.0;

  b[0] = 0.0;
  b[1] = 1.0;
  b[2] = 0.0;

  double result = InnerProduct(a, b);
  EXPECT_DOUBLE_EQ(0.0, result);
}

/**
 * @brief Test for OuterProduct result zero
 */
TEST(Vector, OuterProductZero) {
  const size_t N = 3;
  s2e::math::Vector<N> a;
  s2e::math::Vector<N> b;

  a[0] = 1.0;
  a[1] = 0.0;
  a[2] = 0.0;

  b[0] = 1.0;
  b[1] = 0.0;
  b[2] = 0.0;

  s2e::math::Vector<3> result = OuterProduct(a, b);

  for (size_t i = 0; i < N; i++) {
    EXPECT_DOUBLE_EQ(0.0, result[i]);
  }
}

/**
 * @brief Test for OuterProduct result X axis
 */
TEST(Vector, OuterProductX) {
  const size_t N = 3;
  s2e::math::Vector<N> a;
  s2e::math::Vector<N> b;

  a[0] = 0.0;
  a[1] = 0.0;
  a[2] = 1.0;

  b[0] = 0.0;
  b[1] = 1.0;
  b[2] = 0.0;

  s2e::math::Vector<3> result = OuterProduct(a, b);

  EXPECT_DOUBLE_EQ(-1.0, result[0]);
  EXPECT_DOUBLE_EQ(0.0, result[1]);
  EXPECT_DOUBLE_EQ(0.0, result[2]);
}

/**
 * @brief Test for OuterProduct result Y axis
 */
TEST(Vector, OuterProductY) {
  const size_t N = 3;
  s2e::math::Vector<N> a;
  s2e::math::Vector<N> b;

  a[0] = 0.0;
  a[1] = 0.0;
  a[2] = 1.0;

  b[0] = 1.0;
  b[1] = 0.0;
  b[2] = 0.0;

  s2e::math::Vector<3> result = OuterProduct(a, b);

  EXPECT_DOUBLE_EQ(0.0, result[0]);
  EXPECT_DOUBLE_EQ(1.0, result[1]);
  EXPECT_DOUBLE_EQ(0.0, result[2]);
}

/**
 * @brief Test for OuterProduct result Z axis
 */
TEST(Vector, OuterProductZ) {
  const size_t N = 3;
  s2e::math::Vector<N> a;
  s2e::math::Vector<N> b;

  a[0] = 1.0;
  a[1] = 0.0;
  a[2] = 0.0;

  b[0] = 0.0;
  b[1] = 1.0;
  b[2] = 0.0;

  s2e::math::Vector<3> result = OuterProduct(a, b);

  EXPECT_DOUBLE_EQ(0.0, result[0]);
  EXPECT_DOUBLE_EQ(0.0, result[1]);
  EXPECT_DOUBLE_EQ(1.0, result[2]);
}

/**
 * @brief Test for CalcNorm
 */
TEST(Vector, CalcNorm) {
  const size_t N = 10;
  s2e::math::Vector<N> v(1.0);

  double norm = v.CalcNorm();

  // Check nondestructive
  for (size_t i = 0; i < N; i++) {
    EXPECT_DOUBLE_EQ(1.0, v[i]);
  }

  EXPECT_DOUBLE_EQ(sqrt(double(N)), norm);
}

/**
 * @brief Test for Normalize
 */
TEST(Vector, Normalize) {
  const size_t N = 5;
  s2e::math::Vector<N> v(1.0);

  s2e::math::Vector<N> normalized = v.CalcNormalizedVector();

  for (size_t i = 0; i < N; i++) {
    // Check nondestructive
    EXPECT_DOUBLE_EQ(1.0, v[i]);
    // Check nondestructive
    EXPECT_DOUBLE_EQ(1.0 / sqrt(double(N)), normalized[i]);
  }
}

/**
 * @brief Test for CalcAngleTwoVectors result 90 deg
 */
TEST(Vector, CalcAngleTwoVectors90deg) {
  const size_t N = 3;
  s2e::math::Vector<N> a;
  s2e::math::Vector<N> b;

  a[0] = 1.0;
  a[1] = 0.0;
  a[2] = 0.0;

  b[0] = 0.0;
  b[1] = 1.0;
  b[2] = 0.0;

  double angle_rad = CalcAngleTwoVectors_rad(a, b);

  EXPECT_DOUBLE_EQ(90.0 * s2e::math::deg_to_rad, angle_rad);
}

/**
 * @brief Test for CalcAngleTwoVectors result 0 deg
 */
TEST(Vector, CalcAngleTwoVectors0deg) {
  const size_t N = 3;
  s2e::math::Vector<N> a;

  a[0] = 1.0;
  a[1] = 0.0;
  a[2] = 0.0;

  double angle_rad = CalcAngleTwoVectors_rad(a, a);

  EXPECT_DOUBLE_EQ(0.0 * s2e::math::deg_to_rad, angle_rad);
}

/**
 * @brief Test for CalcAngleTwoVectors result 45 deg
 */
TEST(Vector, CalcAngleTwoVectors45deg) {
  const size_t N = 3;
  s2e::math::Vector<N> a;
  s2e::math::Vector<N> b;

  a[0] = 0.0;
  a[1] = 1.0;
  a[2] = 1.0;

  b[0] = 0.0;
  b[1] = 1.0;
  b[2] = 0.0;

  double angle_rad = CalcAngleTwoVectors_rad(a, b);

  EXPECT_DOUBLE_EQ(45.0 * s2e::math::deg_to_rad, angle_rad);
}

/**
 * @brief Test for GenerateOrthogonalUnitVector
 */
TEST(Vector, GenerateOrthogonalUnitVector) {
  const size_t N = 3;
  s2e::math::Vector<N> a(1.0);

  s2e::math::Vector<N> b = GenerateOrthogonalUnitVector(a);

  double angle_rad = CalcAngleTwoVectors_rad(a, b);

  EXPECT_DOUBLE_EQ(90.0 * s2e::math::deg_to_rad, angle_rad);
}
