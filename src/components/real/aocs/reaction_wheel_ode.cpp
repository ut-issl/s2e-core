/*
 * @file reaction_wheel_ode.cpp
 * @brief Ordinary differential equation of angular velocity of reaction wheel with first-order lag
 */
#include "reaction_wheel_ode.hpp"

#include <library/utilities/macros.hpp>

ReactionWheelOde::ReactionWheelOde(const double step_width_s, const double initial_angular_velocity, const double target_angular_velocity,
                                   const libra::Vector<3> lag_coefficients)
    : OrdinaryDifferentialEquation<1>(step_width_s), lag_coefficients_(lag_coefficients), target_angular_velocity_rad_s_(target_angular_velocity) {
  this->Setup(0.0, libra::Vector<1>(initial_angular_velocity));
}

void ReactionWheelOde::DerivativeFunction(double x, const libra::Vector<1> &state, libra::Vector<1> &rhs) {
  // FIXME: fix this function
  UNUSED(x);
  UNUSED(state);
  //	double zero_deviation;
  //	double one_deviation;
  //	zero_deviation = this->state()[0];
  //	one_deviation = this->state()[1];
  //	printf("zerodev %f\n", zero_deviation);
  //	printf("onedev %f\n", one_deviation);

  // rhs[0] =0.8*(target_angular_velocity_rad_s_ - this->state()[0]) /
  // (first_order_lag_)-second_order_coef_*this->state()[1]; rhs[0] =
  // 0.00030016*(target_angular_velocity_rad_s_ - this->state()[0]) +
  // 0.9899*this->state()[1]-0.0245; rhs[0] = lag_coefficients_[2] * this->state()[1] +
  // lag_coefficients_[1] * (target_angular_velocity_rad_s_ - this->state()[0]) +
  // lag_coefficients_[0];
  // First-order-lag
  rhs[0] = (target_angular_velocity_rad_s_ - this->GetState()[0]) / (lag_coefficients_[0]);
  // Only target
  // rhs[0]   = (target_angular_velocity_rad_s_);
}
