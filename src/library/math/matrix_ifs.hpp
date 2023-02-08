/**
 * @file matrix_ifs.hpp
 * @brief Matrix class to handle math matrix with template
 */

#ifndef S2E_LIBRARY_MATH_MATRIX_IFS_H_
#define S2E_LIBRARY_MATH_MATRIX_IFS_H_

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
  if (!is_valid_range_(row, column)) {
    throw std::invalid_argument("Argument exceeds the range of matrix.");
  }
  return matrix_[row][column];
}

template <size_t R, size_t C, typename T>
const T& Matrix<R, C, T>::operator()(size_t row, size_t column) const {
  if (!is_valid_range_(row, column)) {
    throw std::invalid_argument("Argument exceeds the range of matrix.");
  }
  return matrix_[row][column];
}

template <size_t R, size_t C, typename T>
bool Matrix<R, C, T>::is_valid_range_(size_t row, size_t column) {
  return (row < R && column < C);
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_MATRIX_IFS_H_
