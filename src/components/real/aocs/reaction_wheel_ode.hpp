/*
 * @file reaction_wheel_ode.hpp
 * @brief Ordinary differential equation of angular velocity of reaction wheel with first-order lag
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_ODE_HPP_
#define S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_ODE_HPP_

#include <library/math/ordinary_differential_equation.hpp>
#include <library/math/vector.hpp>
#include <vector>

/*
 * @file RwOde
 * @brief Ordinary differential equation of angular velocity of reaction wheel with first-order lag
 */
class RwOde : public libra::OrdinaryDifferentialEquation<1> {
 public:
  /**
   * @fn RwOde
   * @brief Constructor
   * @param [in] step_width_s: Step width for OrdinaryDifferentialEquation calculation
   * @param [in] initial_angular_velocity: Initial angular velocity [rad/s]
   * @param [in] target_angular_velocity: Target angular velocity [rad/s]
   * @param [in] lag_coefficients: coefficients for first order lag
   */
  RwOde(const double step_width_s, const double initial_angular_velocity, const double target_angular_velocity,
        const libra::Vector<3> lag_coefficients);

  /**
   * @fn DerivativeFunction
   * @brief Definition of the difference equation (Override function of OrdinaryDifferentialEquation class)
   * @param [in] x: Independent variable (e.g. time)
   * @param [in] state: State vector
   * @param [out] rhs: Differentiated value of state vector
   */
  void DerivativeFunction(double x, const libra::Vector<1>& state, libra::Vector<1>& rhs) override;

  /**
   * @fn getAngularVelocity
   * @brief Return current angular velocity of RW rotor [rad/s]
   */
  double getAngularVelocity(void) const { return this->GetState()[0]; }

  /**
   * @fn setTargetAngularVelocity
   * @brief Set target angular velocity [rad/s]
   */
  void setTargetAngularVelocity(double angular_velocity) { target_angular_velocity_rad_s_ = angular_velocity; }

  /**
   * @fn setLagCoef
   * @brief Set lag coefficients
   */
  void setLagCoef(libra::Vector<3> lag_coefficients) { lag_coefficients_ = lag_coefficients; }

 private:
  RwOde(double step_width_s);             //!< Prohibit calling constructor
  libra::Vector<3> lag_coefficients_;     //!< Coefficients for the first order lag
  double target_angular_velocity_rad_s_;  //!< Target angular velocity [rad/s]
};

#endif  // S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_ODE_HPP_
