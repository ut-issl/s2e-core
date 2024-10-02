/**
 * @file matrix_template_functions.hpp
 * @brief Matrix class to handle math matrix with template
 */

#ifndef S2E_LIBRARY_MATH_MATRIX_TEMPLATE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_MATRIX_TEMPLATE_FUNCTIONS_HPP_

#include <cmath>
#include <iostream>  // for cout

namespace s2e::math {

template <size_t R, size_t C, typename T>
Matrix<R, C, T>::Matrix(const T& n) {
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      matrix_[i][j] = n;
    }
  }
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T>& Matrix<R, C, T>::operator+=(const Matrix<R, C, T>& m) {
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      matrix_[i][j] += m.matrix_[i][j];
    }
  }
  return *this;
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T>& Matrix<R, C, T>::operator*=(const T& n) {
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      matrix_[i][j] *= n;
    }
  }
  return *this;
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T>& Matrix<R, C, T>::operator/=(const T& n) {
  return operator*=(1.0 / n);
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T>& Matrix<R, C, T>::operator-=(const Matrix<R, C, T>& m) {
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      matrix_[i][j] -= m.matrix_[i][j];
    }
  }
  return *this;
}

template <size_t R, size_t C, typename T>
void Matrix<R, C, T>::FillUp(const T& t) {
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      matrix_[i][j] = t;
    }
  }
}

template <size_t R, size_t C, typename T>
T Matrix<R, C, T>::CalcTrace() const {
  T trace = 0.0;

  if (R != C) return trace;

  for (size_t i = 0; i < R; ++i) {
    trace += matrix_[i][i];
  }
  return trace;
}

template <size_t R, size_t C, typename T>
void Matrix<R, C, T>::Print(char delimiter, std::ostream& stream) const {
  for (size_t i = 0; i < R; ++i) {
    stream << matrix_[i][0];
    for (size_t j = 1; j < C; ++j) {
      stream << delimiter << matrix_[i][j];
    }
    stream << std::endl;
  }
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T> operator+(const Matrix<R, C, T>& lhs, const Matrix<R, C, T>& rhs) {
  Matrix<R, C, T> temp;
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      temp[i][j] = lhs[i][j] + rhs[i][j];
    }
  }
  return temp;
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T> operator-(const Matrix<R, C, T>& lhs, const Matrix<R, C, T>& rhs) {
  Matrix<R, C, T> temp;
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      temp[i][j] = lhs[i][j] - rhs[i][j];
    }
  }
  return temp;
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T> operator*(const T& rhs, const Matrix<R, C, T>& lhs) {
  Matrix<R, C, T> temp;
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      temp[i][j] = rhs * lhs[i][j];
    }
  }
  return temp;
}

template <size_t R, size_t C1, size_t C2, typename T>
const Matrix<R, C2, T> operator*(const Matrix<R, C1, T>& lhs, const Matrix<C1, C2, T>& rhs) {
  Matrix<R, C2, T> temp(0);
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C2; ++j) {
      for (size_t k = 0; k < C1; ++k) {
        temp[i][j] += lhs[i][k] * rhs[k][j];
      }
    }
  }
  return temp;
}

template <size_t R, size_t C, typename T>
const Matrix<C, R, T> Matrix<R, C, T>::Transpose() const {
  Matrix<C, R, T> temp;
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      temp[j][i] = matrix_[i][j];
    }
  }
  return temp;
}

template <size_t R, typename T>
Matrix<R, R, T> MakeIdentityMatrix() {
  Matrix<R, R, T> m(0.0);
  for (size_t i = 0; i < R; ++i) {
    m[i][i] = 1.0;
  }
  return m;
}

template <size_t R, typename T>
Matrix<R, R, T> MakeRotationMatrixX(const double& theta_rad) {
  Matrix<R, R, T> m = MakeIdentityMatrix<R, T>();

  m[1][1] = cos(theta_rad);
  m[1][2] = sin(theta_rad);
  m[2][1] = -sin(theta_rad);
  m[2][2] = cos(theta_rad);
  return m;
}

template <size_t R, typename T>
Matrix<R, R, T> MakeRotationMatrixY(const double& theta_rad) {
  Matrix<R, R, T> m = MakeIdentityMatrix<R, T>();

  m[0][0] = cos(theta_rad);
  m[0][2] = -sin(theta_rad);
  m[2][0] = sin(theta_rad);
  m[2][2] = cos(theta_rad);
  return m;
}

template <size_t R, typename T>
Matrix<R, R, T> MakeRotationMatrixZ(const double& theta_rad) {
  Matrix<R, R, T> m = MakeIdentityMatrix<R, T>();

  m[0][0] = cos(theta_rad);
  m[0][1] = sin(theta_rad);
  m[1][0] = -sin(theta_rad);
  m[1][1] = cos(theta_rad);
  return m;
}

}  // namespace s2e::math

#endif  // S2E_LIBRARY_MATH_MATRIX_TEMPLATE_FUNCTIONS_HPP_
