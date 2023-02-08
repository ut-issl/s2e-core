/**
 * @file ordinary_differential_equation_tfs.hpp
 * @brief Class for Ordinary Difference Equation (template functions)
 */
#ifndef S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_TFS_HPP_
#define S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_TFS_HPP_

namespace libra {

template <size_t N>
ODE<N>::ODE(double step_width) : x_(0.0), state_(0.0), rhs_(0.0), step_width_(step_width) {}

template <size_t N>
void ODE<N>::setup(double init_x, const Vector<N>& init_cond) {
  x_ = init_x;
  state_ = init_cond;
}

template <size_t N>
ODE<N>& ODE<N>::operator++() {
  Update();
  return *this;
}

template <size_t N>
void ODE<N>::Update() {
  RHS(x_, state_, rhs_);  // Current derivative calculation

  // 4th order Runge-Kutta method
  Vector<N> k1(rhs_);
  k1 *= step_width_;
  Vector<N> k2(state_.dim());
  RHS(x_ + 0.5 * step_width_, state_ + 0.5 * k1, k2);
  k2 *= step_width_;
  Vector<N> k3(state_.dim());
  RHS(x_ + 0.5 * step_width_, state_ + 0.5 * k2, k3);
  k3 *= step_width_;
  Vector<N> k4(state_.dim());
  RHS(x_ + step_width_, state_ + k3, k4);
  k4 *= step_width_;

  state_ += (1.0 / 6.0) * (k1 + 2.0 * (k2 + k3) + k4);  // Update state vector
  x_ += step_width_;                                    // Update independent variable
}

template <size_t N>
void ODE<N>::setStepWidth(double new_step) {
  step_width_ = new_step;
}
}  // namespace libra

#endif  // S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_TFS_HPP_
