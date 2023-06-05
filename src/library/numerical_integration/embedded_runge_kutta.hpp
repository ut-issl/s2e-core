/**
 * @file embedded_runge_kutta.hpp
 * @brief Base Class for Embedded Runge-Kutta method
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_EMBEDDED_RUNGE_KUTTA_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_EMBEDDED_RUNGE_KUTTA_HPP_

#include "runge_kutta.hpp"

namespace libra::numerical_integrator {

/**
 * @class EmbeddedRungeKutta
 * @brief Class for Embedded Runge-Kutta method
 */
template <size_t N>
class EmbeddedRungeKutta : public RungeKutta<N> {
 public:
  /**
   * @fn EmbeddedRungeKutta
   * @brief Constructor
   * @param [in] step_width_s: Step width [s]
   */
  EmbeddedRungeKutta(const double step_width_s, const InterfaceOde<N>& ode) : RungeKutta<N>(step_width_s, ode) {}

  /**
   * @fn Integrate
   * @brief Update the state vector with the numerical integration with multiple order to evaluate the error
   */
  virtual void Integrate();

  /**
   * @fn ControlStepWidth
   * @brief Step width control
   * @param[in] error_tolerance: Error tolerance (epsilon in the equation)
   */
  void ControlStepWidth(const double error_tolerance);

  /**
   * @fn GetLocalTruncationError
   * @return Norm of estimated local truncation error
   */
  inline double GetLocalTruncationError() const { return local_truncation_error_; }

 protected:
  // Parameters should be defined by child class
  std::vector<double> higher_order_weights_;  //!< Weights vector for higher order approximation

  // Error
  double local_truncation_error_;  //!< Norm of estimated local truncation error
};

}  // namespace libra::numerical_integrator

#include "embedded_runge_kutta_template.hpp"

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_EMBEDDED_RUNGE_KUTTA_HPP_
