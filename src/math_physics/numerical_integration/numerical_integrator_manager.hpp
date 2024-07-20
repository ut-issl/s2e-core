/**
 * @file numerical_integrator_manager.hpp
 * @brief Class to manage all numerical integration
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATION_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATION_HPP_

#include <memory>

#include "dormand_prince_5.hpp"
#include "runge_kutta_4.hpp"
#include "runge_kutta_fehlberg.hpp"

namespace libra::numerical_integration {

/**
 * @enum NumericalIntegrationMethod
 * @brief Numerical Integration method
 */
enum class NumericalIntegrationMethod {
  kRk4 = 0,  //!< 4th order Runge-Kutta
  kRkf,      //!< Runge-Kutta-Fehlberg
  kDp5,      //!< 5th order Dormand and Prince
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
   * @param [in] step_width: Step width
   */
  NumericalIntegratorManager(const double step_width, const InterfaceOde<N>& ode,
                             const NumericalIntegrationMethod method = NumericalIntegrationMethod::kRk4) {
    switch (method) {
      case NumericalIntegrationMethod::kRk4:
        integrator_ = std::make_shared<RungeKutta4<N>>(step_width, ode);
        break;
      case NumericalIntegrationMethod::kRkf:
        integrator_ = std::make_shared<RungeKuttaFehlberg<N>>(step_width, ode);
        break;
      case NumericalIntegrationMethod::kDp5:
        integrator_ = std::make_shared<DormandPrince5<N>>(step_width, ode);
        break;
      default:
        integrator_ = std::make_shared<RungeKutta4<N>>(step_width, ode);
        break;
    }
  }

  ~NumericalIntegratorManager(){}

  inline std::shared_ptr<NumericalIntegrator<N>> GetIntegrator() const { return integrator_; }

 private:
  std::shared_ptr<NumericalIntegrator<N>> integrator_;
};

}  // namespace libra::numerical_integration

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATION_HPP_
