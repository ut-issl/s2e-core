/*
 * @file reaction_wheel_ode.hpp
 * @brief Ordinary differential equation of angular velocity of reaction wheel with first-order lag
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_ODE_HPP_
#define S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_ODE_HPP_

#include <math_physics/math/ordinary_differential_equation.hpp>

/*
 * @file ReactionWheelOde
 * @brief Ordinary differential equation of angular velocity of reaction wheel with first-order lag
 */
class ReactionWheelOde : public math::OrdinaryDifferentialEquation<1> {
 public:
  /**
   * @fn ReactionWheelOde
   * @brief Constructor
   * @param [in] step_width_s: Step width for OrdinaryDifferentialEquation calculation
   * @param [in] velocity_limit_rad_s: RW angular velocity limit [rad/s]
   * @param [in] initial_angular_velocity_rad_s: Initial angular velocity [rad/s]
   */
  ReactionWheelOde(const double step_width_s, const double velocity_limit_rad_s, const double initial_angular_velocity_rad_s = 0.0);

  /**
   * @fn SetAngularAcceleration_rad_s2
   * @brief Set output acceleration
   * @param[in] acceleration_rad_s2: Output acceleration [rad/s2]
   */
  inline void SetAngularAcceleration_rad_s2(const double acceleration_rad_s2) { angular_acceleration_rad_s2_ = acceleration_rad_s2; }

  /**
   * @fn SetAngularVelocityLimit_rad_s
   * @brief Set angular velocity limit
   * @param[in] velocity_limit_rad_s: Velocity limit [rad/s]
   */
  inline void SetAngularVelocityLimit_rad_s(const double velocity_limit_rad_s) { velocity_limit_rad_s_ = velocity_limit_rad_s; }

  /**
   * @fn GetAngularVelocity_rad_s
   * @brief Return current angular velocity of RW rotor [rad/s]
   */
  inline double GetAngularVelocity_rad_s(void) const { return GetState()[0]; }

 private:
  ReactionWheelOde(double step_width_s);  //!< Prohibit calling constructor

  /**
   * @fn DerivativeFunction
   * @brief Definition of the difference equation (Override function of OrdinaryDifferentialEquation class)
   * @param [in] x: Independent variable (e.g. time)
   * @param [in] state: State vector
   * @param [out] rhs: Differentiated value of state vector
   */
  void DerivativeFunction(double x, const math::Vector<1>& state, math::Vector<1>& rhs) override;

  double velocity_limit_rad_s_;
  double angular_acceleration_rad_s2_ = 0.0;  //!< Angular acceleration [rad/s2]
};

#endif  // S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_ODE_HPP_
