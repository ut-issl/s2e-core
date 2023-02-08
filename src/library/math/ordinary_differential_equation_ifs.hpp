/**
 * @file ordinary_differential_equation_ifs.hpp
 * @brief Class for Ordinary Difference Equation (inline functions)
 */

#ifndef S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_IFS_HPP_
#define S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_IFS_HPP_

namespace libra {

template <size_t N>
ODE<N>::~ODE() {}

template <size_t N>
double ODE<N>::step_width() const {
  return step_width_;
}

template <size_t N>
double ODE<N>::x() const {
  return x_;
}

template <size_t N>
const Vector<N>& ODE<N>::state() const {
  return state_;
}

template <size_t N>
double ODE<N>::operator[](int n) const {
  return state_[n];
}

template <size_t N>
const Vector<N>& ODE<N>::rhs() const {
  return rhs_;
}

template <size_t N>
libra::Vector<N>& ODE<N>::state() {
  return state_;
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_IFS_HPP_
