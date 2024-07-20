/**
 * @file ode_examples.hpp
 * @brief Examples for implementation of Ordinary Differential Equations
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_

#include "../../utilities/macros.hpp"
#include "interface_ode.hpp"

namespace numerical_integration {

class ExampleLinearOde : public InterfaceOde<1> {
 public:
  Vector<1> DerivativeFunction(const double time_s, const Vector<1>& state) const {
    UNUSED(time_s);
    UNUSED(state);

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
    UNUSED(time_s);
    UNUSED(state);

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
    UNUSED(time_s);

    Vector<2> output(0.0);
    output[0] = state[1];
    output[1] = 0.0;
    return output;
  }
};

/**
 * @class Example2dTwoBodyOrbit
 * @brief Class for position/velocity equation of motion implementation example
 */
class Example2dTwoBodyOrbitOde : public InterfaceOde<4> {
 public:
  virtual Vector<4> DerivativeFunction(const double time_s, const Vector<4>& state) const {
    UNUSED(time_s);

    Vector<4> output(0.0);
    output[0] = state[2];
    output[1] = state[3];
    double denominator = pow(state[0] * state[0] + state[1] * state[1], 3.0 / 2.0);
    double inverse_square;
    if (abs(denominator) <= DBL_EPSILON) {
      inverse_square = 0.0;  // singular
    } else {
      inverse_square = 1.0 / denominator;
    }
    output[2] = -1.0 * state[0] * inverse_square;
    output[3] = -1.0 * state[1] * inverse_square;
    return output;
  }
};

}  // namespace numerical_integration

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_s
