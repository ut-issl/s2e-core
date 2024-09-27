/**
 * @file attitude_rk4.cpp
 * @brief Class to calculate spacecraft attitude with Runge-Kutta method
 */
#include "attitude_rk4.hpp"

#include <iostream>
#include <logger/log_utility.hpp>
#include <sstream>
#include <utilities/macros.hpp>

namespace s2e::dynamics::attitude {

AttitudeRk4::AttitudeRk4(const s2e::math::Vector<3>& angular_velocity_b_rad_s, const s2e::math::Quaternion& quaternion_i2b,
                         const s2e::math::Matrix<3, 3>& inertia_tensor_kgm2, const s2e::math::Vector<3>& torque_b_Nm, const double propagation_step_s,
                         const std::string& simulation_object_name)
    : Attitude(inertia_tensor_kgm2, simulation_object_name) {
  angular_velocity_b_rad_s_ = angular_velocity_b_rad_s;
  quaternion_i2b_ = quaternion_i2b;
  torque_b_Nm_ = torque_b_Nm;
  propagation_step_s_ = propagation_step_s;
  current_propagation_time_s_ = 0.0;
  angular_momentum_reaction_wheel_b_Nms_ = s2e::math::Vector<3>(0.0);
  previous_inertia_tensor_kgm2_ = inertia_tensor_kgm2_;
  inverse_inertia_tensor_ = CalcInverseMatrix(inertia_tensor_kgm2_);
  CalcAngularMomentum();
}

AttitudeRk4::~AttitudeRk4() {}

void AttitudeRk4::SetParameters(const MonteCarloSimulationExecutor& mc_simulator) {
  Attitude::SetParameters(mc_simulator);
  GetInitializedMonteCarloParameterVector(mc_simulator, "angular_velocity_b_rad_s", angular_velocity_b_rad_s_);

  // TODO: Consider the following calculation is needed here?
  current_propagation_time_s_ = 0.0;
  angular_momentum_reaction_wheel_b_Nms_ = s2e::math::Vector<3>(0.0);  //!< Consider how to handle this variable
  CalcAngularMomentum();
}

void AttitudeRk4::Propagate(const double end_time_s) {
  if (!is_calc_enabled_) return;

  s2e::math::Matrix<3, 3> dot_inertia_tensor = (1.0 / (end_time_s - current_propagation_time_s_)) * (inertia_tensor_kgm2_ - previous_inertia_tensor_kgm2_);
  torque_inertia_tensor_change_b_Nm_ = dot_inertia_tensor * angular_velocity_b_rad_s_;
  inverse_inertia_tensor_ = CalcInverseMatrix(inertia_tensor_kgm2_);

  while (end_time_s - current_propagation_time_s_ - propagation_step_s_ > 1.0e-6) {
    RungeKuttaOneStep(current_propagation_time_s_, propagation_step_s_);
    current_propagation_time_s_ += propagation_step_s_;
  }
  RungeKuttaOneStep(current_propagation_time_s_, end_time_s - current_propagation_time_s_);

  // Update information
  current_propagation_time_s_ = end_time_s;
  previous_inertia_tensor_kgm2_ = inertia_tensor_kgm2_;
  CalcAngularMomentum();
}

s2e::math::Vector<7> AttitudeRk4::AttitudeDynamicsAndKinematics(s2e::math::Vector<7> x, double t) {
  UNUSED(t);

  s2e::math::Vector<7> dxdt;

  s2e::math::Vector<3> omega_b;
  for (int i = 0; i < 3; i++) {
    omega_b[i] = x[i];
  }
  s2e::math::Vector<3> angular_momentum_total_b_Nms = (previous_inertia_tensor_kgm2_ * omega_b) + angular_momentum_reaction_wheel_b_Nms_;
  s2e::math::Vector<3> rhs =
      inverse_inertia_tensor_ * (torque_b_Nm_ - s2e::math::OuterProduct(omega_b, angular_momentum_total_b_Nms) - torque_inertia_tensor_change_b_Nm_);

  for (int i = 0; i < 3; ++i) {
    dxdt[i] = rhs[i];
  }

  s2e::math::Vector<4> quaternion_i2b;
  for (int i = 0; i < 4; i++) {
    quaternion_i2b[i] = x[i + 3];
  }

  s2e::math::Vector<4> d_quaternion = 0.5 * CalcAngularVelocityMatrix(omega_b) * quaternion_i2b;

  for (int i = 0; i < 4; i++) {
    dxdt[i + 3] = d_quaternion[i];
  }

  return dxdt;
}

void AttitudeRk4::RungeKuttaOneStep(double t, double dt) {
  s2e::math::Vector<7> x;
  for (int i = 0; i < 3; i++) {
    x[i] = angular_velocity_b_rad_s_[i];
  }
  for (int i = 0; i < 4; i++) {
    x[i + 3] = quaternion_i2b_[i];
  }

  s2e::math::Vector<7> k1, k2, k3, k4;
  s2e::math::Vector<7> xk2, xk3, xk4;

  k1 = AttitudeDynamicsAndKinematics(x, t);
  xk2 = x + (dt / 2.0) * k1;

  k2 = AttitudeDynamicsAndKinematics(xk2, (t + dt / 2.0));
  xk3 = x + (dt / 2.0) * k2;

  k3 = AttitudeDynamicsAndKinematics(xk3, (t + dt / 2.0));
  xk4 = x + dt * k3;

  k4 = AttitudeDynamicsAndKinematics(xk4, (t + dt));

  s2e::math::Vector<7> next_x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

  for (int i = 0; i < 3; i++) {
    angular_velocity_b_rad_s_[i] = next_x[i];
  }
  for (int i = 0; i < 4; i++) {
    quaternion_i2b_[i] = next_x[i + 3];
  }
  quaternion_i2b_.Normalize();
}

} // namespace s2e::dynamics::attitude
