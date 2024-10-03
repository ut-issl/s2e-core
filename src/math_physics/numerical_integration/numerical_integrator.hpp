/**
 * @file numerical_integrator.hpp
 * @brief Abstract class for General numerical integrator
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATOR_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATOR_HPP_

#include <vector>

#include "../math/vector.hpp"
#include "interface_ode.hpp"

namespace s2e::numerical_integration {

/**
 * @class NumericalIntegrator
 * @brief Abstract class for General numerical integrator
 */
template <size_t N>
class NumericalIntegrator {
 public:
  /**
   * @fn NumericalIntegrator
   * @brief Constructor
   * @param [in] step_width: Step width. The unit is depending on the independent variable
   * @param [in] ode: Ordinary differential equation
   */
  inline NumericalIntegrator(const double step_width, const InterfaceOde<N>& ode)
      : step_width_(step_width), ode_(ode), current_independent_variable_(0.0), current_state_(0.0), previous_state_(0.0) {}
  /**
   * @fn ~NumericalIntegrator
   * @brief Destructor
   */
  inline virtual ~NumericalIntegrator(){};

  /**
   * @fn Integrate
   * @brief Update the state vector with the numerical integration
   */
  virtual void Integrate() = 0;

  /**
   * @fn SetState
   * @brief Set state information
   */
  inline void SetState(const double independent_variable, const math::Vector<N>& state) {
    current_independent_variable_ = independent_variable;
    current_state_ = state;
    previous_state_ = state;
  }

  /**
   * @fn GetState
   * @brief Return current state vector
   */
  inline const math::Vector<N>& GetState() const { return current_state_; }

  /**
   * @fn CalcInterpolationState
   * @brief Calculate interpolation state.
   * @param [in] sigma: Sigma value (0 < sigma < 1) for interpolation
   * @return : interpolated state x(t0 + sigma * h)
   */
  virtual math::Vector<N> CalcInterpolationState(const double sigma) const = 0;

 protected:
  // Settings
  double step_width_;  //!< Step width. The unit is depending on the independent variable

  // States
  const InterfaceOde<N>& ode_;           //!< Ordinary differential equation
  double current_independent_variable_;  //!< Latest value of independent variable
  math::Vector<N> current_state_;        //!< Latest state vector
  math::Vector<N> previous_state_;       //!< Previous state vector
};

}  // namespace s2e::numerical_integration

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATOR_HPP_
