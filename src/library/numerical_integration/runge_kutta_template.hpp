/**
 * @file runge_kutta_template.hpp
 * @brief Implementations for RungeKutta class
 */
#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_

#include "./runge_kutta.hpp"

namespace libra {

template <size_t N>
void RungeKutta<N>::Integrate() {
  CalcSlope();

  this->previous_state_ = this->current_state_;
  for (size_t i = 0; i < number_of_stages_; i++) {
    this->current_state_ = this->current_state_ + weights_[i] * this->step_width_s_ * slope_[i];
  }
  this->current_time_s_ += this->step_width_s_;
}

template <size_t N>
void RungeKutta<N>::CalcSlope() {
  slope_.assign(number_of_stages_, Vector<N>(0.0));

  for (size_t i = 0; i < number_of_stages_; i++) {
    Vector<N> state = this->current_state_;
    for (size_t j = 0; j < i; j++) {
      state = state + rk_matrix_[i][j] * this->step_width_s_ * slope_[j];
    }
    double time_s = this->current_time_s_ + nodes_[i] * this->step_width_s_;
    slope_[i] = this->ode_.DerivativeFunction(time_s, state);
  }
}

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_
