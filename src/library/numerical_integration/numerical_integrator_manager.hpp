/**
 * @file numerical_integrator_manager.hpp
 * @brief Class to manage all numerical integration
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATION_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATION_HPP_

#include "runge_kutta_4.hpp"
#include "runge_kutta_fehlberg.hpp"

namespace libra::numerical_integrator {

/**
 * @enum NumericalIntegrationMethod
 * @brief Numerical Integration method
 */
enum class NumericalIntegrationMethod {
  kRk4 = 0,  //!< 4th order Runge-Kutta
  kRkf,      //!< Runge-Kutta-Fehlberg
};

/**
 * @class NumericalIntegratorManager
 * @brief Class to manage all numerical integration
 */
template <size_t N>
class NumericalIntegratorManager {
 public:
  /**
   * @fn NumericalIntegrator
   * @brief Constructor
   * @param [in] step_width_s: Step width
   */
  NumericalIntegratorManager(const double step_width_s, const InterfaceOde<N>& ode,
                             const NumericalIntegrationMethod method = NumericalIntegrationMethod::kRk4) {
    switch (method) {
      case NumericalIntegrationMethod::kRk4:
        integrator_ = new RungeKutta4<N>(step_width_s, ode);
        break;
      case NumericalIntegrationMethod::kRkf:
        integrator_ = new RungeKuttaFehlberg<N>(step_width_s, ode);
        break;
      default:
        integrator_ = new RungeKutta4<N>(step_width_s, ode);
        break;
    }
  }

  ~NumericalIntegratorManager() { delete integrator_; }

  inline NumericalIntegrator<N>* GetIntegrator() const { return integrator_; }

 private:
  NumericalIntegrator<N>* integrator_;
};

}  // namespace libra::numerical_integrator

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATION_HPP_
