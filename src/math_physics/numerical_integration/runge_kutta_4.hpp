/**
 * @file runge_kutta_4.hpp
 * @brief Class for Classical 4th order Runge-Kutta method
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_HPP_

#include <utilities/macros.hpp>

#include "runge_kutta.hpp"

namespace numerical_integration {

/**
 * @class RungeKutta4
 * @brief Class for Classical 4th order Runge-Kutta method
 */
template <size_t N>
class RungeKutta4 : public RungeKutta<N> {
 public:
  /**
   * @fn RungeKutta
   * @brief Constructor
   * @param [in] step_width: Step width
   */
  RungeKutta4(const double step_width, const InterfaceOde<N>& ode) : RungeKutta<N>(step_width, ode) {
    // Classical 4th order Runge-Kutta (4-order, 4-stage)
    this->number_of_stages_ = 4;
    this->approximation_order_ = 4;
    this->nodes_.assign(this->number_of_stages_, 0.0);
    this->weights_.assign(this->number_of_stages_, 0.0);
    this->rk_matrix_.assign(this->number_of_stages_, std::vector<double>(this->number_of_stages_, 0.0));

    this->nodes_[1] = this->nodes_[2] = 0.5;
    this->nodes_[3] = 1.0;

    this->weights_[0] = this->weights_[3] = 1.0 / 6.0;
    this->weights_[1] = this->weights_[2] = 1.0 / 3.0;

    this->rk_matrix_[1][0] = this->rk_matrix_[2][1] = 0.5;
    this->rk_matrix_[3][2] = 1.0;

    this->CalcSlope();
  }

  // We did not implement the interpolation for RK4
  math::Vector<N> CalcInterpolationState(const double sigma) const override {
    UNUSED(sigma);
    return this->current_state_;
  }
};

}  // namespace numerical_integration

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_HPP_
