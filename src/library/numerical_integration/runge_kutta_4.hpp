/**
 * @file runge_kutta.hpp
 * @brief Class for Runge-Kutta-4 method
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_HPP_

#include "runge_kutta.hpp"

namespace libra {

/**
 * @class RungeKutta
 * @brief Class for General Runge-Kutta method
 */
template <size_t N>
class RungeKutta4 : public RungeKutta<N> {
 public:
  /**
   * @fn RungeKutta
   * @brief Constructor
   * @param [in] step_width_s: Step width
   */
  RungeKutta4(const double step_width_s) : RungeKutta<N>(step_width_s) {
    // Classical 4th order Runge-Kutta
    this->stage_ = 4;
    this->c_.assign(this->stage_, 0.0);
    this->b_.assign(this->stage_, 0.0);
    this->a_.assign(this->stage_, std::vector<double>(this->stage_, 0.0));

    this->c_[1] = this->c_[2] = 0.5;
    this->c_[3] = 1.0;

    this->b_[0] = this->b_[3] = 1.0 / 6.0;
    this->b_[1] = this->b_[2] = 1.0 / 3.0;

    this->a_[1][0] = this->a_[2][1] = 0.5;
    this->a_[3][2] = 1.0;
  }
};

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_HPP_
