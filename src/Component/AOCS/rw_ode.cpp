/*
 * @file rw_ode.cpp
 * @brief Ordinary differential equation of angular velocity of reaction wheel with first-order lag
 */
#include "rw_ode.hpp"

#include <Library/utils/Macros.hpp>

using namespace libra;

RwOde::RwOde(double step_width, double init_angular_velocity, double target_angular_velocity, Vector<3> lag_coef) ODE<1>(step_width),
    lag_coef_(lag_coef), kInitAngularVelocity_(init_angular_velocity), target_angular_velocity_(target_angular_velocity) {
  this->setup(0.0, Vector<1>(init_angular_velocity));
}

double RwOde::getAngularVelocity(void) const {
  double angular_velocity = this->state()[0];
  return angular_velocity;
}

void RwOde::RHS(double x, const Vector<1> &state, Vector<1> &rhs) {
  UNUSED(x);
  // First-order-lag
  rhs[0] = -1 / lag_coef_[0] * this->state()[0] + target_angular_velocity_ / lag_coef_[0];
}

void RwOde::setTargetAngularVelocity(double angular_velocity) { target_angular_velocity_ = angular_velocity; }

void RwOde::setLagCoef(const Vector<3> lag_coef) { lag_coef_ = lag_coef; }
