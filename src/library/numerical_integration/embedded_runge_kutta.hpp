/**
 * @file embedded_runge_kutta.hpp
 * @brief Class for Embedded Runge-Kutta method
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_EMBEDDED_RUNGE_KUTTA_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_EMBEDDED_RUNGE_KUTTA_HPP_

#include "runge_kutta.hpp"

namespace libra {

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
  EmbeddedRungeKutta(const double step_width_s) : RungeKutta<N>(step_width_s) {}

  /**
   * @fn Integrate
   * @brief Update the state vector with the numerical integration
   */
  virtual void Integrate();

  /**
   * @fn ControlStepWidth
   * @brief Step width control
   * @param[in] error_tolerance: Error tolerance (epsilon in the equation)
   */
  void ControlStepWidth(double error_tolerance);

  /**
   * @fn GetLocalTruncationError
   * @return Norm of estimated local truncation error
   */
  inline double GetLocalTruncationError() const { return local_truncation_error_; }

 private:
  // Parameters
  std::vector<double> higher_order_weights_;  //!< Weights vector for higher order approximation

  // Error
  double local_truncation_error_;  //!< Norm of estimated local truncation error
};

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_EMBEDDED_RUNGE_KUTTA_HPP_
