/**
 * @file ordinary_differential_equation_template_functions.hpp
 * @brief Class for Ordinary Difference Equation (template functions)
 */
#ifndef S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_TEMPLATE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_TEMPLATE_FUNCTIONS_HPP_

namespace libra {

template <size_t N>
ODE<N>::ODE(double step_width) : independent_variable_(0.0), state_(0.0), derivative_(0.0), step_width_(step_width) {}

template <size_t N>
void ODE<N>::Setup(double initial_independent_variable, const Vector<N>& initial_state) {
  independent_variable_ = initial_independent_variable;
  state_ = initial_state;
}

template <size_t N>
ODE<N>& ODE<N>::operator++() {
  Update();
  return *this;
}

template <size_t N>
void ODE<N>::Update() {
  RHS(independent_variable_, state_, derivative_);  // Current derivative calculation

  // 4th order Runge-Kutta method
  Vector<N> k1(derivative_);
  k1 *= step_width_;
  Vector<N> k2(state_.dim());
  RHS(independent_variable_ + 0.5 * step_width_, state_ + 0.5 * k1, k2);
  k2 *= step_width_;
  Vector<N> k3(state_.dim());
  RHS(independent_variable_ + 0.5 * step_width_, state_ + 0.5 * k2, k3);
  k3 *= step_width_;
  Vector<N> k4(state_.dim());
  RHS(independent_variable_ + step_width_, state_ + k3, k4);
  k4 *= step_width_;

  state_ += (1.0 / 6.0) * (k1 + 2.0 * (k2 + k3) + k4);  // Update state vector
  independent_variable_ += step_width_;                 // Update independent variable
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_TEMPLATE_FUNCTIONS_HPP_
