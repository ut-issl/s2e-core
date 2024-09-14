/**
 * @file runge_kutta_template.hpp
 * @brief Implementations for RungeKutta class
 */
#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_

#include "./runge_kutta.hpp"

namespace s2e::numerical_integration {

template <size_t N>
void RungeKutta<N>::Integrate() {
  CalcSlope();

  this->previous_state_ = this->current_state_;
  for (size_t i = 0; i < number_of_stages_; i++) {
    this->current_state_ = this->current_state_ + weights_[i] * this->step_width_ * slope_[i];
  }
  this->current_independent_variable_ += this->step_width_;
}

template <size_t N>
void RungeKutta<N>::CalcSlope() {
  slope_.assign(number_of_stages_, s2e::math::Vector<N>(0.0));

  for (size_t i = 0; i < number_of_stages_; i++) {
    s2e::math::Vector<N> state = this->current_state_;
    for (size_t j = 0; j < i; j++) {
      state = state + rk_matrix_[i][j] * this->step_width_ * slope_[j];
    }
    double independent_variable = this->current_independent_variable_ + nodes_[i] * this->step_width_;
    slope_[i] = this->ode_.DerivativeFunction(independent_variable, state);
  }
}

}  // namespace s2e::numerical_integration

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_TEMPLATE_HPP_
