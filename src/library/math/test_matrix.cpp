/**
 * @file test_quaternion.cpp
 * @brief Test codes for Quaternion class with GoogleTest
 */
#include <gtest/gtest.h>

#include "constants.hpp"
#include "matrix.hpp"

/**
 * @brief Test for constructor with number
 */
TEST(Matrix, ConstructorWithNumber) {
  const size_t R = 6;
  const size_t C = 3;
  double initialize_value = 2.0;
  libra::Matrix<R, C> m(initialize_value);

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      EXPECT_DOUBLE_EQ(initialize_value, m[r][c]);
      EXPECT_DOUBLE_EQ(initialize_value, m(r, c));
    }
  }
}

/**
 * @brief Test for GetRowLength and GetColumnLength
 */
TEST(Matrix, GetLength) {
  const size_t R = 6;
  const size_t C = 3;
  libra::Matrix<R, C> m;

  EXPECT_EQ(R, m.GetRowLength());
  EXPECT_EQ(C, m.GetColumnLength());
}

/**
 * @brief Test for operator+=
 */
TEST(Matrix, OperatorPlusEqual) {
  const size_t R = 6;
  const size_t C = 3;
  double initialize_value = 2.0;
  libra::Matrix<R, C> m(initialize_value);
  libra::Matrix<R, C> adding;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      adding[r][c] = r * c;
    }
  }

  m += adding;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      // Check nondestructive
      EXPECT_DOUBLE_EQ(r * c, adding[r][c]);
      // Check result
      EXPECT_DOUBLE_EQ(initialize_value + r * c, m[r][c]);
    }
  }
}

/**
 * @brief Test for operator-=
 */
TEST(Matrix, OperatorMinusEqual) {
  const size_t R = 6;
  const size_t C = 3;
  double initialize_value = 2.0;
  libra::Matrix<R, C> m(initialize_value);
  libra::Matrix<R, C> subtracting;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      subtracting[r][c] = r * c;
    }
  }

  m -= subtracting;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      // Check nondestructive
      EXPECT_DOUBLE_EQ(r * c, subtracting[r][c]);
      // Check result
      EXPECT_DOUBLE_EQ(initialize_value - r * c, m[r][c]);
    }
  }
}

/**
 * @brief Test for operator*=
 */
TEST(Matrix, OperatorMultiplyEqual) {
  const size_t R = 6;
  const size_t C = 3;

  libra::Matrix<R, C> m;
  double multiplying = 2.0;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      m[r][c] = r * c;
    }
  }

  m *= multiplying;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      // Check result
      EXPECT_DOUBLE_EQ(2.0 * r * c, m[r][c]);
    }
  }
}

/**
 * @brief Test for operator/=
 */
TEST(Matrix, OperatorDivideEqual) {
  const size_t R = 6;
  const size_t C = 3;

  libra::Matrix<R, C> m;
  double dividing = 2.0;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      m[r][c] = r * c;
    }
  }

  m /= dividing;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      // Check result
      EXPECT_DOUBLE_EQ(r * c / 2.0, m[r][c]);
    }
  }
}

/**
 * @brief Test for FillUp
 */
TEST(Matrix, FillUp) {
  const size_t R = 6;
  const size_t C = 3;
  double value = 3.0;

  libra::Matrix<R, C> m;

  FillUp(m, value);

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      // Check result
      EXPECT_DOUBLE_EQ(value, m[r][c]);
    }
  }
}

/**
 * @brief Test for CalcTrace
 */
TEST(Matrix, CalcTrace) {
  const size_t N = 6;
  libra::Matrix<N, N> m;

  for (size_t r = 0; r < N; r++) {
    for (size_t c = 0; c < N; c++) {
      m[r][c] = r * c;
    }
  }

  double trace = CalcTrace(m);

  // Check nondestructive
  for (size_t r = 0; r < N; r++) {
    for (size_t c = 0; c < N; c++) {
      EXPECT_DOUBLE_EQ(r * c, m[r][c]);
    }
  }

  // Check
  double trace_check = 0.0;
  for (size_t i = 0; i < N; i++) {
    trace_check += i * i;
  }
  EXPECT_DOUBLE_EQ(trace_check, trace);
}

/**
 * @brief Test for operator+
 */
TEST(Matrix, OperatorPlus) {
  const size_t R = 6;
  const size_t C = 3;
  double initialize_value = -2.0;
  libra::Matrix<R, C> m(initialize_value);
  libra::Matrix<R, C> adding;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      adding[r][c] = r * c;
    }
  }

  libra::Matrix<R, C> added = m + adding;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      // Check nondestructive
      EXPECT_DOUBLE_EQ(initialize_value, m[r][c]);
      EXPECT_DOUBLE_EQ(r * c, adding[r][c]);
      // Check result
      EXPECT_DOUBLE_EQ(initialize_value + r * c, added[r][c]);
    }
  }
}

/**
 * @brief Test for operator-
 */
TEST(Matrix, OperatorMinus) {
  const size_t R = 6;
  const size_t C = 3;
  double initialize_value = 0.6;
  libra::Matrix<R, C> m(initialize_value);
  libra::Matrix<R, C> subtracting;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      subtracting[r][c] = r * c;
    }
  }

  libra::Matrix<R, C> subtracted = m - subtracting;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      // Check nondestructive
      EXPECT_DOUBLE_EQ(initialize_value, m[r][c]);
      EXPECT_DOUBLE_EQ(r * c, subtracting[r][c]);
      // Check result
      EXPECT_DOUBLE_EQ(initialize_value - r * c, subtracted[r][c]);
    }
  }
}

/**
 * @brief Test for operator* scalar
 */
TEST(Matrix, OperatorMultiplyScalar) {
  const size_t R = 6;
  const size_t C = 3;

  libra::Matrix<R, C> m;
  double multiplying = 0.3;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      m[r][c] = r * c;
    }
  }

  libra::Matrix<R, C> subtracted = multiplying * m;

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      // Check nondestructive
      EXPECT_DOUBLE_EQ(r * c, m[r][c]);
      // Check result
      EXPECT_DOUBLE_EQ(0.3 * r * c, subtracted[r][c]);
    }
  }
}

/**
 * @brief Test for operator* Matrix
 */
TEST(Matrix, OperatorMultiplyMatrix) {
  const size_t R = 2;
  const size_t C = 3;

  libra::Matrix<R, C> a;
  libra::Matrix<C, R> b;

  a[0][0] = 1.0;
  a[0][1] = 2.0;
  a[0][2] = 3.0;
  a[1][0] = 4.0;
  a[1][1] = 5.0;
  a[1][2] = 6.0;

  b[0][0] = 1.0;
  b[0][1] = 2.0;
  b[1][0] = 3.0;
  b[1][1] = 4.0;
  b[2][0] = 5.0;
  b[2][1] = 6.0;

  libra::Matrix<R, R> result = a * b;

  EXPECT_DOUBLE_EQ(22.0, result[0][0]);
  EXPECT_DOUBLE_EQ(28.0, result[0][1]);
  EXPECT_DOUBLE_EQ(49.0, result[1][0]);
  EXPECT_DOUBLE_EQ(64.0, result[1][1]);
}

/**
 * @brief Test for Transpose
 */
TEST(Matrix, Transpose) {
  const size_t R = 6;
  const size_t C = 3;

  libra::Matrix<R, C> m;
  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      m[r][c] = r * c;
    }
  }

  libra::Matrix<C, R> transposed = Transpose(m);

  for (size_t r = 0; r < R; r++) {
    for (size_t c = 0; c < C; c++) {
      // Check nondestructive
      EXPECT_DOUBLE_EQ(r * c, m[r][c]);
      // Check result
      EXPECT_DOUBLE_EQ(r * c, transposed[c][r]);
    }
  }
}

/**
 * @brief Test for Unitalize
 */
TEST(Matrix, Unitalize) {
  const size_t N = 6;

  libra::Matrix<N, N> m;
  for (size_t r = 0; r < N; r++) {
    for (size_t c = 0; c < N; c++) {
      m[r][c] = r * c;
    }
  }

  libra::Matrix<N, N> unitalized = Unitalize(m);

  for (size_t r = 0; r < N; r++) {
    for (size_t c = 0; c < N; c++) {
      // Check nondestructive (Currently, this is destructive)
      if (r == c) {
        EXPECT_DOUBLE_EQ(1.0, m[r][c]);
      } else {
        EXPECT_DOUBLE_EQ(0.0, m[r][c]);
      }
      // Check result
      if (r == c) {
        EXPECT_DOUBLE_EQ(1.0, unitalized[r][c]);
      } else {
        EXPECT_DOUBLE_EQ(0.0, unitalized[r][c]);
      }
    }
  }
}

/**
 * @brief Test for MakeIdentityMatrix
 */
TEST(Matrix, MakeIdentityMatrix) {
  const size_t N = 6;

  libra::Matrix<N, N> m = libra::MakeIdentityMatrix<N>();

  for (size_t r = 0; r < N; r++) {
    for (size_t c = 0; c < N; c++) {
      if (r == c) {
        EXPECT_DOUBLE_EQ(1.0, m[r][c]);
      } else {
        EXPECT_DOUBLE_EQ(0.0, m[r][c]);
      }
    }
  }
}

/**
 * @brief Test for MakeRotationMatrixX
 */
TEST(Matrix, MakeRotationMatrixX) {
  const size_t N = 3;
  double theta_rad = -45.0 * libra::deg_to_rad;

  libra::Matrix<N, N> m = libra::MakeRotationMatrixX(theta_rad);

  EXPECT_DOUBLE_EQ(1.0, m[0][0]);
  EXPECT_DOUBLE_EQ(0.0, m[0][1]);
  EXPECT_DOUBLE_EQ(0.0, m[0][2]);
  EXPECT_DOUBLE_EQ(0.0, m[1][0]);
  EXPECT_DOUBLE_EQ(cos(theta_rad), m[1][1]);
  EXPECT_DOUBLE_EQ(sin(theta_rad), m[1][2]);
  EXPECT_DOUBLE_EQ(0.0, m[2][0]);
  EXPECT_DOUBLE_EQ(-sin(theta_rad), m[2][1]);
  EXPECT_DOUBLE_EQ(cos(theta_rad), m[2][2]);
}

/**
 * @brief Test for MakeRotationMatrixY
 */
TEST(Matrix, MakeRotationMatrixY) {
  const size_t N = 3;
  double theta_rad = 120.0 * libra::deg_to_rad;

  libra::Matrix<N, N> m = libra::MakeRotationMatrixY(theta_rad);

  EXPECT_DOUBLE_EQ(cos(theta_rad), m[0][0]);
  EXPECT_DOUBLE_EQ(0.0, m[0][1]);
  EXPECT_DOUBLE_EQ(-sin(theta_rad), m[0][2]);
  EXPECT_DOUBLE_EQ(0.0, m[1][0]);
  EXPECT_DOUBLE_EQ(1.0, m[1][1]);
  EXPECT_DOUBLE_EQ(0.0, m[1][2]);
  EXPECT_DOUBLE_EQ(sin(theta_rad), m[2][0]);
  EXPECT_DOUBLE_EQ(0.0, m[2][1]);
  EXPECT_DOUBLE_EQ(cos(theta_rad), m[2][2]);
}

/**
 * @brief Test for MakeRotationMatrixZ
 */
TEST(Matrix, MakeRotationMatrixZ) {
  const size_t N = 3;
  double theta_rad = 30.0 * libra::deg_to_rad;

  libra::Matrix<N, N> m = libra::MakeRotationMatrixZ(theta_rad);

  EXPECT_DOUBLE_EQ(cos(theta_rad), m[0][0]);
  EXPECT_DOUBLE_EQ(sin(theta_rad), m[0][1]);
  EXPECT_DOUBLE_EQ(0.0, m[0][2]);
  EXPECT_DOUBLE_EQ(-sin(theta_rad), m[1][0]);
  EXPECT_DOUBLE_EQ(cos(theta_rad), m[1][1]);
  EXPECT_DOUBLE_EQ(0.0, m[1][2]);
  EXPECT_DOUBLE_EQ(0.0, m[2][0]);
  EXPECT_DOUBLE_EQ(0.0, m[2][1]);
  EXPECT_DOUBLE_EQ(1.0, m[2][2]);
}
