/*
 * @file reaction_wheel_ode.cpp
 * @brief Ordinary differential equation of angular velocity of reaction wheel with first-order lag
 */
#include "reaction_wheel_ode.hpp"

#include <library/utilities/Macros.hpp>

using namespace libra;

RwOde::RwOde(double step_width, double init_angular_velocity, double target_angular_velocity, Vector<3> lag_coef)
    : ODE<1>(step_width), lag_coef_(lag_coef), kInitAngularVelocity_(init_angular_velocity), target_angular_velocity_(target_angular_velocity) {
  this->setup(0.0, Vector<1>(init_angular_velocity));
}

double RwOde::getAngularVelocity(void) const {
  double angular_velocity = this->state()[0];
  return angular_velocity;
}

void RwOde::RHS(double x, const Vector<1> &state, Vector<1> &rhs) {
  // FIXME: fix this function
  UNUSED(x);
  UNUSED(state);
  //	double zero_deviation;
  //	double one_deviation;
  //	zero_deviation = this->state()[0];
  //	one_deviation = this->state()[1];
  //	printf("zerodev %f\n", zero_deviation);
  //	printf("onedev %f\n", one_deviation);

  // rhs[0] =0.8*(target_angular_velocity_ - this->state()[0]) /
  // (first_order_lag_)-second_order_coef_*this->state()[1]; rhs[0] =
  // 0.00030016*(target_angular_velocity_ - this->state()[0]) +
  // 0.9899*this->state()[1]-0.0245; rhs[0] = lag_coef_[2] * this->state()[1] +
  // lag_coef_[1] * (target_angular_velocity_ - this->state()[0]) +
  // lag_coef_[0];
  // First-order-lag
  rhs[0] = (target_angular_velocity_ - this->state()[0]) / (lag_coef_[0]);
  // Only target
  // rhs[0]   = (target_angular_velocity_);
}

void RwOde::setTargetAngularVelocity(double angular_velocity) { target_angular_velocity_ = angular_velocity; }

void RwOde::setLagCoef(const Vector<3> lag_coef) { lag_coef_ = lag_coef; }
