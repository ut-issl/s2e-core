/*!
  \file   Matrix_tfs.hpp
  \author TAKISAWA Jun'ichi.
  \date   Sun Oct 24 07:22:34 2010
  \brief  Matrix.hppのtemplate関数実装
*/
#ifndef MATRIX_TFS_HPP_
#define MATRIX_TFS_HPP_

#include <cmath>
#include <iostream>  // for cout

namespace libra {

template <size_t R, size_t C, typename T>
Matrix<R, C, T>::Matrix(const T& n) {
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      matrix_[i][j] = n;
    }
  }
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T>& Matrix<R, C, T>::operator+=(const Matrix<R, C, T>& m) {
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      matrix_[i][j] += m.matrix_[i][j];
    }
  }
  return *this;
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T>& Matrix<R, C, T>::operator*=(const T& n) {
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
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
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      matrix_[i][j] -= m.matrix_[i][j];
    }
  }
  return *this;
}

template <size_t R, size_t C, typename T>
void fill_up(Matrix<R, C, T>& m, const T& t) {
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      m[i][j] = t;
    }
  }
}

template <size_t N, typename T>
T trace(const Matrix<N, N, T>& m) {
  T trace = 0.0;
  for (int i = 0; i < N; ++i) {
    trace += m[i][i];
  }
  return trace;
}

template <size_t R, size_t C, typename T>
void print(const Matrix<R, C, T>& m, char delimiter, std::ostream& stream) {
  for (int i = 0; i < R; ++i) {
    stream << m[i][0];
    for (int j = 1; j < C; ++j) {
      stream << delimiter << m[i][j];
    }
    stream << std::endl;
  }
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T> operator+(const Matrix<R, C, T>& lhs,
                                const Matrix<R, C, T>& rhs) {
  Matrix<R, C, T> temp;
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      temp[i][j] = lhs[i][j] + rhs[i][j];
    }
  }
  return temp;
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T> operator-(const Matrix<R, C, T>& lhs,
                                const Matrix<R, C, T>& rhs) {
  Matrix<R, C, T> temp;
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      temp[i][j] = lhs[i][j] - rhs[i][j];
    }
  }
  return temp;
}

template <size_t R, size_t C, typename T>
const Matrix<R, C, T> operator*(const T& rhs, const Matrix<R, C, T>& lhs) {
  Matrix<R, C, T> temp;
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      temp[i][j] = rhs * lhs[i][j];
    }
  }
  return temp;
}

template <size_t R, size_t C1, size_t C2, typename T>
const Matrix<R, C2, T> operator*(const Matrix<R, C1, T>& lhs,
                                 const Matrix<C1, C2, T>& rhs) {
  Matrix<R, C2, T> temp(0);
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C2; ++j) {
      for (int k = 0; k < C1; ++k) {
        temp[i][j] += lhs[i][k] * rhs[k][j];
      }
    }
  }
  return temp;
}

template <size_t R, size_t C, typename T>
const Matrix<C, R, T> transpose(const Matrix<R, C, T>& m) {
  Matrix<C, R, T> temp;
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      temp[j][i] = m[i][j];
    }
  }
  return temp;
}

template <size_t R, typename T>
Matrix<R, R, T>& unitalize(Matrix<R, R, T>& m) {
  fill_up(m, 0.0);
  for (int i = 0; i < R; ++i) {
    m[i][i] = 1.0;
  }
  return m;
}

template <size_t R, typename T>
Matrix<R, R, T> eye() {
  Matrix<R, R, T> m;
  unitalize(m);
  return m;
}

template <size_t R, typename T>
Matrix<R, R, T> rotx(const double& theta) {
  Matrix<R, R, T> m;
  unitalize(m);
  m[1][1] = cos(theta);
  m[1][2] = sin(theta);
  m[2][1] = -sin(theta);
  m[2][2] = cos(theta);
  return m;
}

template <size_t R, typename T>
Matrix<R, R, T> roty(const double& theta) {
  Matrix<R, R, T> m;
  unitalize(m);
  m[0][0] = cos(theta);
  m[0][2] = -sin(theta);
  m[2][0] = sin(theta);
  m[2][2] = cos(theta);
  return m;
}

template <size_t R, typename T>
Matrix<R, R, T> rotz(const double& theta) {
  Matrix<R, R, T> m;
  unitalize(m);
  m[0][0] = cos(theta);
  m[0][1] = sin(theta);
  m[1][0] = -sin(theta);
  m[1][1] = cos(theta);
  return m;
}

}  // namespace libra

#endif  // MATRIX_TFS_HPP_
