/**
 * @file runge_kutta_fehlberg.hpp
 * @brief Class for Classical Runge-Kutta-Fehlberg method
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_FEHLBERG_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_FEHLBERG_HPP_

#include "embedded_runge_kutta.hpp"

namespace libra::numerical_integration {

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
   * @param [in] step_width: Step width
   */
  RungeKuttaFehlberg(const double step_width, const InterfaceOde<N>& ode);
  /**
   * @fn CalcInterpolationState
   * @brief Calculate interpolation state
   * @param [in] sigma: Sigma value (0 < sigma < 1) for interpolation
   * @return : interpolated state x(t0 + sigma * h)
   */
  Vector<N> CalcInterpolationState(const double sigma);

 private:
  /**
   * @fn CalcInterpolationWeights
   * @brief Calculate weights for interpolation
   * @param [in] sigma: Sigma value (0 < sigma < 1) for interpolation
   * @return : weights for interpolation
   */
  std::vector<double> CalcInterpolationWeights(const double sigma);
};

}  // namespace libra::numerical_integration

#include "runge_kutta_fehlberg_implementation.hpp"

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_FEHLBERG_HPP_
