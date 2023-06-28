/**
 * @file embedded_runge_kutta_implementation.hpp
 * @brief Implementation of Embedded Runge-Kutta method
 */
#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_EMBEDDED_RUNGE_KUTTA_IMPLEMENTATION_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_EMBEDDED_RUNGE_KUTTA_IMPLEMENTATION_HPP_

#include "embedded_runge_kutta.hpp"

namespace libra::numerical_integration {

template <size_t N>
void EmbeddedRungeKutta<N>::Integrate() {
  this->CalcSlope();

  Vector<N> lower_current_state = this->current_state_;   //!< eta in the equation
  Vector<N> higher_current_state = this->current_state_;  //!< eta_hat in the equation
  for (size_t i = 0; i < this->number_of_stages_; i++) {
    lower_current_state = lower_current_state + this->weights_[i] * this->step_width_ * this->slope_[i];
    higher_current_state = higher_current_state + higher_order_weights_[i] * this->step_width_ * this->slope_[i];
  }

  // Error evaluation
  Vector<N> truncation_error = lower_current_state - higher_current_state;
  local_truncation_error_ = truncation_error.CalcNorm();

  // State update
  this->current_state_ = higher_current_state;
  this->current_independent_variable_ += this->step_width_;
}

template <size_t N>
void EmbeddedRungeKutta<N>::ControlStepWidth(const double error_tolerance) {
  double updated_step_width = pow(error_tolerance / local_truncation_error_, 1.0 / ((double)(this->approximation_order_ + 1))) * this->step_width_;
  if (updated_step_width <= 0.0) return;  // TODO: Error handling
  this->step_width_ = updated_step_width;
}

}  // namespace libra::numerical_integration

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_EMBEDDED_RUNGE_KUTTA_IMPLEMENTATION_HPP_
