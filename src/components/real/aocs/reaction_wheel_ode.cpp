/*
 * @file reaction_wheel_ode.cpp
 * @brief Ordinary differential equation of angular velocity of reaction wheel with first-order lag
 */
#include "reaction_wheel_ode.hpp"

#include <utilities/macros.hpp>

ReactionWheelOde::ReactionWheelOde(const double step_width_s, const double velocity_limit_rad_s, const double initial_angular_velocity_rad_s)
    : OrdinaryDifferentialEquation<1>(step_width_s), velocity_limit_rad_s_(velocity_limit_rad_s) {
  this->Setup(0.0, libra::Vector<1>(initial_angular_velocity_rad_s));
}

void ReactionWheelOde::DerivativeFunction(double x, const libra::Vector<1> &state, libra::Vector<1> &rhs) {
  UNUSED(x);
  double angular_velocity_rad_s = state[0];

  // Check velocity limit
  if (angular_velocity_rad_s > velocity_limit_rad_s_) {
    if (angular_acceleration_rad_s2_ > 0.0) {
      angular_acceleration_rad_s2_ = 0.0;
    }
  } else if (angular_velocity_rad_s < -1.0 * velocity_limit_rad_s_) {
    if (angular_acceleration_rad_s2_ < 0.0) {
      angular_acceleration_rad_s2_ = 0.0;
    }
  }

  rhs[0] = angular_acceleration_rad_s2_;
}
