/**
 * @file ordinary_differential_equation_template_functions.hpp
 * @brief Class for Ordinary Differential Equation (template functions)
 */
#ifndef S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_TEMPLATE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_TEMPLATE_FUNCTIONS_HPP_

namespace libra {

template <size_t N>
OrdinaryDifferentialEquation<N>::OrdinaryDifferentialEquation(double step_width_s)
    : independent_variable_(0.0), state_(0.0), derivative_(0.0), step_width_s_(step_width_s) {}

template <size_t N>
void OrdinaryDifferentialEquation<N>::Setup(double initial_independent_variable, const Vector<N>& initial_state) {
  independent_variable_ = initial_independent_variable;
  state_ = initial_state;
}

template <size_t N>
OrdinaryDifferentialEquation<N>& OrdinaryDifferentialEquation<N>::operator++() {
  Update();
  return *this;
}

template <size_t N>
void OrdinaryDifferentialEquation<N>::Update() {
  DerivativeFunction(independent_variable_, state_, derivative_);  // Current derivative calculation

  // 4th order Runge-Kutta method
  Vector<N> k1(derivative_);
  k1 *= step_width_s_;
  Vector<N> k2((double)state_.GetLength());
  DerivativeFunction(independent_variable_ + 0.5 * step_width_s_, state_ + 0.5 * k1, k2);
  k2 *= step_width_s_;
  Vector<N> k3((double)state_.GetLength());
  DerivativeFunction(independent_variable_ + 0.5 * step_width_s_, state_ + 0.5 * k2, k3);
  k3 *= step_width_s_;
  Vector<N> k4((double)state_.GetLength());
  DerivativeFunction(independent_variable_ + step_width_s_, state_ + k3, k4);
  k4 *= step_width_s_;

  state_ += (1.0 / 6.0) * (k1 + 2.0 * (k2 + k3) + k4);  // Update state vector
  independent_variable_ += step_width_s_;               // Update independent variable
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_TEMPLATE_FUNCTIONS_HPP_
