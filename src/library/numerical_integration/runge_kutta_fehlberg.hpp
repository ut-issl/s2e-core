/**
 * @file runge_kutta_fehlberg.hpp
 * @brief Class for Classical Runge-Kutta-Fehlberg method
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_FEHLBERG_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_FEHLBERG_HPP_

#include "embedded_runge_kutta.hpp"

namespace libra {

/**
 * @class RungeKuttaFehlberg
 * @brief Class for Classical Runge-Kutta-Fehlberg method
 */
template <size_t N>
class RungeKuttaFehlberg : public EmbeddedRungeKutta<N> {
 public:
  /**
   * @fn RungeKuttaFehlberg
   * @brief Constructor
   * @param [in] step_width_s: Step width
   */
  RungeKuttaFehlberg(const double step_width_s, const InterfaceOde<N>& ode) : EmbeddedRungeKutta<N>(step_width_s, ode) {
    // p=4th/q=5th order Runge-Kutta-Fehlberg (6-stage)
    this->number_of_stages_ = 6;
    this->approximation_order_ = 4;
    this->nodes_.assign(this->number_of_stages_, 0.0);
    this->weights_.assign(this->number_of_stages_, 0.0);
    this->higher_order_weights_.assign(this->number_of_stages_, 0.0);
    this->rk_matrix_.assign(this->number_of_stages_, std::vector<double>(this->number_of_stages_, 0.0));

    this->nodes_[1] = 1.0 / 4.0;
    this->nodes_[2] = 3.0 / 8.0;
    this->nodes_[3] = 12.0 / 13.0;
    this->nodes_[4] = 1.0;
    this->nodes_[5] = 1.0 / 2.0;

    this->higher_order_weights_[0] = 16.0 / 135.0;
    this->higher_order_weights_[1] = 0.0;
    this->higher_order_weights_[2] = 6656.0 / 12825.0;
    this->higher_order_weights_[3] = 28561.0 / 56430.0;
    this->higher_order_weights_[4] = -9.0 / 50.0;
    this->higher_order_weights_[5] = 2.0 / 55.0;

    this->weights_[0] = 25.0 / 216.0;
    this->weights_[1] = 0.0;
    this->weights_[2] = 1408.0 / 2565.0;
    this->weights_[3] = 2197.0 / 4104.0;
    this->weights_[4] = -1.0 / 5.0;
    this->weights_[5] = 0.0;

    this->rk_matrix_[1][0] = 1.0 / 4.0;

    this->rk_matrix_[2][0] = 3.0 / 32.0;
    this->rk_matrix_[2][1] = 9.0 / 32.0;

    this->rk_matrix_[3][0] = 1932.0 / 2197.0;
    this->rk_matrix_[3][1] = -7200.0 / 2197.0;
    this->rk_matrix_[3][2] = 7296.0 / 2197.0;

    this->rk_matrix_[4][0] = 439.0 / 216.0;
    this->rk_matrix_[4][1] = -8.0;
    this->rk_matrix_[4][2] = 3680.0 / 513.0;
    this->rk_matrix_[4][3] = -845.0 / 4104.0;

    this->rk_matrix_[5][0] = -8.0 / 27.0;
    this->rk_matrix_[5][1] = 2.0;
    this->rk_matrix_[5][2] = -3544.0 / 2565.0;
    this->rk_matrix_[5][3] = 1859.0 / 4104.0;
    this->rk_matrix_[5][4] = -11.0 / 40.0;
  }
};

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_FEHLBERG_HPP_
