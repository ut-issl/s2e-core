/**
 * @file ode_examples.hpp
 * @brief Examples for implementation of Ordinary Differential Equations
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_

#include "interface_ode.hpp"

namespace libra {

class ExampleLinearOde : public InterfaceOde<1> {
 public:
  Vector<1> DerivativeFunction(const double time_s, const Vector<1>& state) const {
    Vector<1> output(1.0);
    return output;
  }
};

/**
 * @class ExampleQuadraticOde
 * @brief Class for simple quadratic ODE implementation example
 */
class ExampleQuadraticOde : public InterfaceOde<1> {
 public:
  /**
   * @fn DerivativeFunction
   * @brief Override function to define the difference equation
   * @param [in] time_s: Time as independent variable
   * @param [in] state: State vector
   * @return Differentiated value of state vector
   */
  virtual Vector<1> DerivativeFunction(const double time_s, const Vector<1>& state) const {
    Vector<1> output(0.0);
    output[0] = 2.0 * time_s;
    return output;
  }
};

/**
 * @class Example1dPositionVelocityOde
 * @brief Class for position/velocity equation of motion implementation example
 */
class Example1dPositionVelocityOde : public InterfaceOde<2> {
 public:
  virtual Vector<2> DerivativeFunction(const double time_s, const Vector<2>& state) const {
    Vector<2> output(0.0);
    output[0] = state[1];
    output[1] = 0.0;
    return output;
  }
};

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_s
