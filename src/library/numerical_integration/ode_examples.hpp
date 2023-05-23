/**
 * @file ode_examples.hpp
 * @brief Examples for implementation of Ordinary Differential Equations
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_

#include "runge_kutta_4.hpp"

namespace libra {

/**
 * @class ExampleLinearOde
 * @brief Class for simple linear ODE implementation example
 */
class ExampleLinearOde : public RungeKutta4<1> {
 public:
  /**
   * @fn ExampleLinearOde
   * @brief Constructor
   * @param [in] step_width_s: Step width
   */
  inline ExampleLinearOde(const double step_width_s) : RungeKutta4<1>(step_width_s) {}

 protected:
  /**
   * @fn DerivativeFunction
   * @brief Override function to define the difference equation
   * @param [in] time_s: Time as independent variable
   * @param [in] state: State vector
   * @return Differentiated value of state vector
   */
  virtual Vector<1> DerivativeFunction(const double time_s, const Vector<1>& state) {
    Vector<1> output(1.0);
    return output;
  }
};

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_s