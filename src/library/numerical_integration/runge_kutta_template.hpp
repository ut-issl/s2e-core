/**
 * @file runge_kutta.cpp
 * @brief Class for General Runge-Kutta method
 */
#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_

#include "./runge_kutta.hpp"

namespace libra {
template <size_t N>
RungeKutta<N>::RungeKutta(const double step_width_s) : step_width_s_(step_width_s), current_time_s_(0.0), current_state_(0.0) {
  // Classical 4th order Runge-Kutta
  order_ = 4;
  stage_ = 4;
  k_.assign(stage_, Vector<N>(0.0));
  c_.assign(stage_, 0.0);
  b_.assign(stage_, 0.0);
  a_.assign(stage_, std::vector<double>(stage_, 0.0));

  // Set coefficients
  c_[1] = c_[2] = 0.5;
  c_[3] = 1.0;
  b_[0] = b_[3] = 1.0 / 6.0;
  b_[1] = b_[2] = 1.0 / 3.0;
  a_[1][0] = a_[2][1] = 0.5;
  a_[3][2] = 1.0;
}

template <size_t N>
Vector<N> RungeKutta<N>::DerivativeFunction(const double time_s, const Vector<N>& state) {
  Vector<N> output(1.0);

  return output;
}

template <size_t N>
void RungeKutta<N>::Integrate() {
  CalcSlope();
  for (size_t i = 0; i < stage_; i++) {
    current_state_ = current_state_ + b_[i] * step_width_s_ * k_[i];
  }
}

template <size_t N>
void RungeKutta<N>::CalcSlope() {
  k_[0] = DerivativeFunction(current_time_s_, current_state_);
  for (size_t i = 1; i < stage_; i++) {
    Vector<N> state = current_state_;
    for (size_t j = 0; j < i; j++) {
      state = state + a_[i][j] * step_width_s_ * k_[j];
    }
    double time_s = current_time_s_ + c_[i] * step_width_s_;
    k_[i] = DerivativeFunction(time_s, state);
  }
}

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_
