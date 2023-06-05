/**
 * @file numerical_integrator.hpp
 * @brief Abstract class for General numerical integrator
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATOR_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATOR_HPP_

#include <vector>

#include "../math/vector.hpp"
#include "interface_ode.hpp"

namespace libra::numerical_integrator {

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
   * @param [in] step_width_s: Step width [s]
   * @param [in] ode: Ordinary differential equation
   */
  inline NumericalIntegrator(const double step_width_s, const InterfaceOde<N>& ode)
      : step_width_s_(step_width_s), ode_(ode), current_time_s_(0.0), current_state_(0.0), previous_state_(0.0) {}
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
  inline void SetState(const double time_s, const Vector<N>& state) {
    previous_state_ = state;
    current_state_ = state;
    current_time_s_ = time_s;
  }

  /**
   * @fn GetState
   * @brief Return current state vector
   */
  inline const Vector<N>& GetState() const { return current_state_; }

 protected:
  // Settings
  double step_width_s_;  //!< Step width [s]

  // States
  const InterfaceOde<N>& ode_;  //!< Ordinary differential equation
  double current_time_s_;       //!< Latest value of independent variable
  Vector<N> current_state_;     //!< Latest state vector
  Vector<N> previous_state_;    //!< Previous state vector
};

}  // namespace libra::numerical_integrator

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_NUMERICAL_INTEGRATOR_HPP_
