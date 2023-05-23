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
  RungeKutta4(const double step_width_s);

 protected:
  /**
   * @fn SetParameters
   * @brief Override function of set parameters for RK
   */
  void SetParameters() override;

  /**
   * @fn DerivativeFunction
   * @brief Override function to define the difference equation
   * @param [in] time_s: Time as independent variable
   * @param [in] state: State vector
   * @return Differentiated value of state vector
   */
  Vector<N> DerivativeFunction(const double time_s, const Vector<N>& state) override;
};

}  // namespace libra

#include "runge_kutta_4_template.hpp"

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_4_HPP_
