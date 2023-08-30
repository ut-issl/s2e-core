/**
 * @file dormand_prince_5_implementation.hpp
 * @brief Implementation of 5th order Dormand and Prince method
 * @note Ref: J. R. Dormand and P. J. Prince, "Runge-Kutta Triples", 1986
 *            O. Montenbruck and E. Gill, "State interpolation for on-board navigation systems", 2001
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_DORMAND_PRINCE_5_IMPLEMENTATION_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_DORMAND_PRINCE_5_IMPLEMENTATION_HPP_

#include "dormand_prince_5.hpp"

namespace libra::numerical_integration {

template <size_t N>
DormandPrince5<N>::DormandPrince5(const double step_width, const InterfaceOde<N>& ode) : EmbeddedRungeKutta<N>(step_width, ode) {
  // p=5th/q=4th order 5th order Dormand and Prince (7-stage)
  this->number_of_stages_ = 7;
  this->approximation_order_ = 5;
  this->nodes_.assign(this->number_of_stages_, 0.0);
  this->weights_.assign(this->number_of_stages_, 0.0);
  this->higher_order_weights_.assign(this->number_of_stages_, 0.0);
  this->rk_matrix_.assign(this->number_of_stages_, std::vector<double>(this->number_of_stages_, 0.0));

  this->nodes_[0] = 0.0;
  this->nodes_[1] = 1.0 / 5.0;
  this->nodes_[2] = 3.0 / 10.0;
  this->nodes_[3] = 4.0 / 5.0;
  this->nodes_[4] = 8.0 / 9.0;
  this->nodes_[5] = 1.0;
  this->nodes_[6] = 1.0;

  this->weights_[0] = 5179.0 / 57600.0;
  this->weights_[1] = 0.0;
  this->weights_[2] = 7571.0 / 16695.0;
  this->weights_[3] = 393.0 / 640.0;
  this->weights_[4] = -92097.0 / 339200.0;
  this->weights_[5] = 187.0 / 2100.0;
  this->weights_[6] = 1.0 / 40.0;

  this->higher_order_weights_[0] = 35.0 / 384.0;
  this->higher_order_weights_[1] = 0.0;
  this->higher_order_weights_[2] = 500.0 / 1113.0;
  this->higher_order_weights_[3] = 125.0 / 192.0;
  this->higher_order_weights_[4] = -2187.0 / 6784.0;
  this->higher_order_weights_[5] = 11.0 / 84.0;
  this->higher_order_weights_[6] = 0.0;

  this->rk_matrix_[1][0] = 1.0 / 5.0;

  this->rk_matrix_[2][0] = 3.0 / 40.0;
  this->rk_matrix_[2][1] = 9.0 / 40.0;

  this->rk_matrix_[3][0] = 44.0 / 45.0;
  this->rk_matrix_[3][1] = -56.0 / 15.0;
  this->rk_matrix_[3][2] = 32.0 / 9.0;

  this->rk_matrix_[4][0] = 19372.0 / 6561.0;
  this->rk_matrix_[4][1] = -25360.0 / 2187.0;
  this->rk_matrix_[4][2] = 64448.0 / 6561.0;
  this->rk_matrix_[4][3] = -212.0 / 729.0;

  this->rk_matrix_[5][0] = 9017.0 / 3168.0;
  this->rk_matrix_[5][1] = -355.0 / 33.0;
  this->rk_matrix_[5][2] = 46732.0 / 5247.0;
  this->rk_matrix_[5][3] = 49.0 / 176.0;
  this->rk_matrix_[5][4] = -5103.0 / 18656.0;

  this->rk_matrix_[6][0] = 35.0 / 384.0;
  this->rk_matrix_[6][1] = 0.0;
  this->rk_matrix_[6][2] = 500.0 / 1113.0;
  this->rk_matrix_[6][3] = 125.0 / 192.0;
  this->rk_matrix_[6][4] = -2187.0 / 6784.0;
  this->rk_matrix_[6][5] = 11.0 / 84.0;

  // Interpolation coefficients
  libra::Vector<5> coefficients_temp;
  coefficients_temp[0] = 11282082432.0;
  coefficients_temp[1] = -32272833064.0;
  coefficients_temp[2] = 34969693132.0;
  coefficients_temp[3] = -13107642775.0;
  coefficients_temp[4] = 157015080.0;
  coefficients_temp = 1.0 / 11282082432.0 * coefficients_temp;
  coefficients_.push_back(coefficients_temp);

  coefficients_temp *= 0.0;
  coefficients_.push_back(coefficients_temp);

  coefficients_temp[0] = 0.0;
  coefficients_temp[1] = -1323431896.0;
  coefficients_temp[2] = 2074956840.0;
  coefficients_temp[3] = -914128567.0;
  coefficients_temp[4] = 15701508.0;
  coefficients_temp = -100.0 / 32700410799.0 * coefficients_temp;
  coefficients_.push_back(coefficients_temp);

  coefficients_temp[0] = 0.0;
  coefficients_temp[1] = -889289856.0;
  coefficients_temp[2] = 2460397220.0;
  coefficients_temp[3] = -1518414297.0;
  coefficients_temp[4] = 94209048.0;
  coefficients_temp = 25.0 / 5641041216.0 * coefficients_temp;
  coefficients_.push_back(coefficients_temp);

  coefficients_temp[0] = 0.0;
  coefficients_temp[1] = -259006536.0;
  coefficients_temp[2] = 687873124.0;
  coefficients_temp[3] = -451824525.0;
  coefficients_temp[4] = 52338360.0;
  coefficients_temp = -2187.0 / 199316789632.0 * coefficients_temp;
  coefficients_.push_back(coefficients_temp);

  coefficients_temp[0] = 0.0;
  coefficients_temp[1] = -361440756.0;
  coefficients_temp[2] = 946554244.0;
  coefficients_temp[3] = -661884105.0;
  coefficients_temp[4] = 106151040.0;
  coefficients_temp = 11.0 / 2467955532.0 * coefficients_temp;
  coefficients_.push_back(coefficients_temp);
}

template <size_t N>
Vector<N> DormandPrince5<N>::CalcInterpolationState(const double sigma) const {
  // Calc k7 (slope after state update)
  Vector<N> state_7 =
      this->previous_state_ + this->step_width_ * (1.0 / 6.0 * this->slope_[0] + 1.0 / 6.0 * this->slope_[4] + 2.0 / 3.0 * this->slope_[5]);
  Vector<N> k7 = this->ode_.DerivativeFunction(this->current_independent_variable_, state_7);

  std::vector<double> interpolation_weights = CalcInterpolationWeights(sigma);

  Vector<N> interpolation_state = this->previous_state_;
  for (size_t i = 0; i < this->number_of_stages_ - 1; i++) {
    interpolation_state = interpolation_state + (sigma * this->step_width_ * interpolation_weights[i]) * this->slope_[i];
  }
  interpolation_state = interpolation_state + sigma * this->step_width_ * (interpolation_weights[6] * k7);
  return interpolation_state;
}

template <size_t N>
std::vector<double> DormandPrince5<N>::CalcInterpolationWeights(const double sigma) const {
  std::vector<double> interpolation_weights;
  interpolation_weights.assign(this->number_of_stages_, 0.0);

  for (size_t stage = 0; stage < this->number_of_stages_ - 1; stage++) {
    for (size_t j = 0; j < 5; j++) {
      interpolation_weights[stage] += pow(sigma, j) * coefficients_[stage][j];
    }
  }
  interpolation_weights[this->number_of_stages_ - 1] =
      sigma * (1.0 - sigma) * (8293050.0 * pow(sigma, 2.0) - 82437520.0 * sigma + 44764047.0) / 29380423.0;
  return interpolation_weights;
}

}  // namespace libra::numerical_integration

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_DORMAND_PRINCE_5_IMPLEMENTATION_HPP_
