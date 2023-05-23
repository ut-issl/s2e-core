/**
 * @file runge_kutta.cpp
 * @brief Class for General Runge-Kutta method
 */
#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_

#include "./runge_kutta.hpp"

namespace libra {

template <size_t N>
void RungeKutta<N>::Integrate() {
  std::vector<Vector<N>> slope = CalcSlope();

  for (size_t i = 0; i < stage_; i++) {
    current_state_ = current_state_ + b_[i] * step_width_s_ * slope[i];
  }
}

template <size_t N>
std::vector<Vector<N>> RungeKutta<N>::CalcSlope() {
  std::vector<Vector<N>> slope;
  slope.assign(stage_, Vector<N>(0.0));

  slope[0] = DerivativeFunction(current_time_s_, current_state_);  // TODO: merge with for state
  for (size_t i = 1; i < stage_; i++) {
    Vector<N> state = current_state_;
    for (size_t j = 0; j < i; j++) {
      state = state + a_[i][j] * step_width_s_ * slope[j];
    }
    double time_s = current_time_s_ + c_[i] * step_width_s_;
    slope[i] = DerivativeFunction(time_s, state);
  }

  return slope;
}

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_
