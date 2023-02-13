/**
 * @file vector_inline_functions.hpp
 * @brief Class for mathematical vector (inline functions)
 */

#ifndef S2E_LIBRARY_MATH_VECTOR_IFS_HPP_
#define S2E_LIBRARY_MATH_VECTOR_IFS_HPP_

#include <stdexcept>  // for invalid_argument.

namespace libra {

template <size_t N, typename T>
Vector<N, T>::Vector() {}

template <size_t N, typename T>
size_t Vector<N, T>::dim() const {
  return N;
}

template <size_t N, typename T>
Vector<N, T>::operator T*() {
  return vector_;
}

template <size_t N, typename T>
Vector<N, T>::operator const T*() const {
  return vector_;
}

template <size_t N, typename T>
T& Vector<N, T>::operator()(std::size_t pos) {
  if (N <= pos) {
    throw std::invalid_argument("Argument exceeds Vector's dimension.");
  }
  return vector_[pos];
}

template <size_t N, typename T>
T Vector<N, T>::operator()(std::size_t pos) const {
  if (N <= pos) {
    throw std::invalid_argument("Argument exceeds Vector's deimension.");
  }
  return vector_[pos];
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_VECTOR_IFS_HPP_
