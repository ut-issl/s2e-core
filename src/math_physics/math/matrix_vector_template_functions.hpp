/**
 * @file matrix_vector_template_functions.hpp
 * @brief Template library for Matrix-Vector calculation
 */

#ifndef S2E_LIBRARY_MATH_MATRIX_VECTOR_TEMPLATE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_MATRIX_VECTOR_TEMPLATE_FUNCTIONS_HPP_

#include <stdexcept>  // for invalid_argument.

namespace libra {

template <size_t R, size_t C, typename TM, typename TC>
Vector<R, TC> operator*(const Matrix<R, C, TM>& matrix, const Vector<C, TC>& vector) {
  Vector<R, TC> temp(0.0);
  for (size_t i = 0; i < R; ++i) {
    for (size_t j = 0; j < C; ++j) {
      temp[i] += matrix[i][j] * vector[j];
    }
  }
  return temp;
}

template <std::size_t N>
Matrix<N, N> CalcInverseMatrix(const Matrix<N, N>& matrix) {
  Matrix<N, N> temp(matrix);
  size_t index[N];
  LuDecomposition(temp, index);

  Matrix<N, N> inverse;
  Vector<N> vector;
  for (size_t i = 0; i < N; ++i) {
    vector.FillUp(0.0);
    vector[i] = 1.0;
    SolveLinearSystemWithLu(temp, index, vector);
    for (size_t j = 0; j < N; ++j) {
      inverse[j][i] = vector[j];
    }
  }
  return inverse;
}

template <std::size_t N>
Matrix<N, N>& LuDecomposition(Matrix<N, N>& a, size_t index[]) {
  double coef[N];
  for (size_t i = 0; i < N; ++i) {
    double biggest = 0.0;
    for (size_t j = 0; j < N; ++j) {
      double temp;
      if ((temp = fabs(a[i][j])) > biggest) {
        biggest = temp;
      }
    }
    if (biggest == 0.0)  // Singular
    {
      throw std::invalid_argument("Given matrix is singular!!");
    }
    coef[i] = 1.0 / biggest;
  }

  for (size_t j = 0; j < N; ++j) {
    for (size_t i = 0; i < j; ++i) {
      double sum = a[i][j];
      for (size_t k = 0; k < i; ++k) {
        sum -= a[i][k] * a[k][j];
      }
      a[i][j] = sum;
    }

    double biggest = 0.0;
    size_t imax = j;
    for (size_t i = j; i < N; ++i) {
      double sum = a[i][j];
      for (size_t k = 0; k < j; ++k) {
        sum -= a[i][k] * a[k][j];
      }
      a[i][j] = sum;

      double temp;
      if ((temp = coef[i] * fabs(sum)) >= biggest) {
        biggest = temp;
        imax = i;
      }
    }

    if (j != imax)  // Pivotting
    {
      for (size_t i = 0; i < N; ++i) {
        double temp = a[imax][i];
        a[imax][i] = a[j][i];
        a[j][i] = temp;
      }
      coef[imax] = coef[j];
    }
    index[j] = imax;

    if (a[j][j] == 0.0)  // Singular
    {
      throw std::invalid_argument("Given matrix is singular!!");
    }
    if (j != N) {
      double temp = 1.0 / a[j][j];
      for (size_t i = j + 1; i < N; ++i) {
        a[i][j] *= temp;
      }
    }
  }
  return a;
}

template <std::size_t N>
Vector<N>& SolveLinearSystemWithLu(const Matrix<N, N>& a, const size_t index[], Vector<N>& b) {
  double sum;
  bool non_zero = false;
  size_t mark = 0;
  for (size_t i = 0; i < N; ++i) {
    size_t ip = index[i];
    sum = b[ip];
    b[ip] = b[i];
    if (non_zero) {
      for (size_t j = mark; j < i; ++j) {
        sum -= a[i][j] * b[j];
      }
    } else if (sum != 0.0) {
      non_zero = true;
      mark = i;
    }
    b[i] = sum;
  }

  for (int i = N - 1; i >= 0; --i) {
    sum = b[i];
    for (size_t j = i + 1; j < N; ++j) {
      sum -= a[i][j] * b[j];
    }
    b[i] = sum / a[i][i];
  }

  return b;
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_MATRIX_VECTOR_TEMPLATE_FUNCTIONS_HPP_
