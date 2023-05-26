/**
 * @file ode_examples.hpp
 * @brief Examples for implementation of Ordinary Differential Equations
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_

#include "runge_kutta_4.hpp"

namespace libra {

class LinearOde {
 public:
  Vector<1> Ode(const double time_s, const Vector<1>& state) {
    Vector<1> output(1.0);
    return output;
  }
};

class ExampleLinearOdeRk4 : public RungeKutta4<1> {
 public:
  /**
   * @fn ExampleLinearOde
   * @brief Constructor
   * @param [in] step_width_s: Step width
   */
  inline ExampleLinearOdeRk4(const double step_width_s, LinearOde& ode) : RungeKutta4<1>(step_width_s), ode_(ode) {}

 protected:
  LinearOde& ode_;
  virtual Vector<1> DerivativeFunction(const double time_s, const Vector<1>& state) {
    return ode_.Ode(time_s, state);
  }
};

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

/**
 * @class ExampleQuadraticOde
 * @brief Class for simple quadratic ODE implementation example
 */
class ExampleQuadraticOde : public RungeKutta4<1> {
 public:
  /**
   * @fn ExampleQuadraticOde
   * @brief Constructor
   * @param [in] step_width_s: Step width
   */
  inline ExampleQuadraticOde(const double step_width_s) : RungeKutta4<1>(step_width_s) {}

 protected:
  /**
   * @fn DerivativeFunction
   * @brief Override function to define the difference equation
   * @param [in] time_s: Time as independent variable
   * @param [in] state: State vector
   * @return Differentiated value of state vector
   */
  virtual Vector<1> DerivativeFunction(const double time_s, const Vector<1>& state) {
    Vector<1> output(0.0);
    output[0] = 2.0 * time_s;
    return output;
  }
};

/**
 * @class Example1dPositionVelocityOde
 * @brief Class for simple quadratic ODE implementation example
 */
class Example1dPositionVelocityOde : public RungeKutta4<2> {
 public:
  /**
   * @fn Example1dPositionVelocityOde
   * @brief Constructor
   * @param [in] step_width_s: Step width
   */
  inline Example1dPositionVelocityOde(const double step_width_s) : RungeKutta4<2>(step_width_s) {}

 protected:
  /**
   * @fn DerivativeFunction
   * @brief Override function to define the difference equation
   * @param [in] time_s: Time as independent variable
   * @param [in] state: State vector
   * @return Differentiated value of state vector
   */
  virtual Vector<2> DerivativeFunction(const double time_s, const Vector<2>& state) {
    Vector<2> output(0.0);
    output[0] = state[1];
    output[1] = 0.0;
    return output;
  }
};

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_EXAMPLE_ODE_HPP_s
