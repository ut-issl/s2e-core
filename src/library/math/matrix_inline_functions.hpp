/**
 * @file matrix_inline_functions.hpp
 * @brief Matrix class to handle math matrix with template
 */

#ifndef S2E_LIBRARY_MATH_MATRIX_INLINE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_MATRIX_INLINE_FUNCTIONS_HPP_

#include <stdexcept>  // for invalid_argument

namespace libra {

template <size_t R, size_t C, typename T>
Matrix<R, C, T>::Matrix() {}

template <size_t R, size_t C, typename T>
size_t Matrix<R, C, T>::row() const {
  return R;
}

template <size_t R, size_t C, typename T>
size_t Matrix<R, C, T>::column() const {
  return C;
}

template <size_t R, size_t C, typename T>
Matrix<R, C, T>::operator TP() {
  return matrix_;
}

template <size_t R, size_t C, typename T>
Matrix<R, C, T>::operator CTP() const {
  return matrix_;
}

template <size_t R, size_t C, typename T>
T& Matrix<R, C, T>::operator()(size_t row, size_t column) {
  if (!ISValidRange(row, column)) {
    throw std::invalid_argument("Argument exceeds the range of matrix.");
  }
  return matrix_[row][column];
}

template <size_t R, size_t C, typename T>
const T& Matrix<R, C, T>::operator()(size_t row, size_t column) const {
  if (!ISValidRange(row, column)) {
    throw std::invalid_argument("Argument exceeds the range of matrix.");
  }
  return matrix_[row][column];
}

template <size_t R, size_t C, typename T>
bool Matrix<R, C, T>::ISValidRange(size_t row, size_t column) {
  return (row < R && column < C);
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_MATRIX_INLINE_FUNCTIONS_HPP_
