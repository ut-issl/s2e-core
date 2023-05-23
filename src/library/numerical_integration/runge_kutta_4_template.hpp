/**
 * @file runge_kutta_4_template.hpp
 * @brief Class for Runge-Kutta-4 method
 */
#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_TEMPLATE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_TEMPLATE_HPP_

#include "runge_kutta_4.hpp"

namespace libra {

template <size_t N>
RungeKutta4<N>::RungeKutta4(const double step_width_s) : RungeKutta<N>(step_width_s) {}

template <size_t N>
void RungeKutta4<N>::SetParameters() {
  // Classical 4th order Runge-Kutta
  this->stage_ = 4;
  this->order_ = 4;
  this->k_.assign(this->stage_, Vector<N>(0.0));
  this->c_.assign(this->stage_, 0.0);
  this->b_.assign(this->stage_, 0.0);
  this->a_.assign(this->stage_, std::vector<double>(this->stage_, 0.0));

  // Set coefficients
  this->c_[1] = this->c_[2] = 0.5;
  this->c_[3] = 1.0;
  this->b_[0] = this->b_[3] = 1.0 / 6.0;
  this->b_[1] = this->b_[2] = 1.0 / 3.0;
  this->a_[1][0] = this->a_[2][1] = 0.5;
  this->a_[3][2] = 1.0;
}

template <size_t N>
Vector<N> RungeKutta4<N>::DerivativeFunction(const double time_s, const Vector<N>& state) {
  Vector<N> output(1.0);
  return output;
}

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_TEMPLATE_HPP_
