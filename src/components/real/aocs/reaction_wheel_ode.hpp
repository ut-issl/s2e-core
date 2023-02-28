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
   * @param [in] step_width: Step width for OrdinaryDifferentialEquation calculation
   * @param [in] init_angular_velocity: Initial angular velocity [rad/s]
   * @param [in] target_angular_velocity: Target angular velocity [rad/s]
   * @param [in] lag_coef: coefficients for first order lag
   */
  RwOde(double step_width, double init_angular_velocity, double target_angular_velocity, libra::Vector<3> lag_coef);

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
  double getAngularVelocity(void) const;

  /**
   * @fn setTargetAngularVelocity
   * @brief Set target angular velocity [rad/s]
   */
  void setTargetAngularVelocity(double angular_velocity);

  /**
   * @fn setFirstOrderLag
   * @brief Set first order lag coefficient (Currently not used)
   */
  void setFirstOrderLag(double first_order_lag);
  /**
   * @fn setSecondOrderCoef
   * @brief Set second order lag coefficient (Currently not used)
   */
  void setSecondOrderCoef(double second_order_coef);
  /**
   * @fn setLagCoef
   * @brief Set lag coefficients
   */
  void setLagCoef(libra::Vector<3> lag_coef);

 private:
  RwOde(double step_width);               //!< Prohibit calling constructor
  libra::Vector<3> lag_coefficients_;     //!< Coefficients for the first order lag
  double target_angular_velocity_rad_s_;  //!< Target angular velocity [rad/s]
};

#endif  // S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_ODE_HPP_
