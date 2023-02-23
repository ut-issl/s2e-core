/**
 * @file attitude_rk4.cpp
 * @brief Class to calculate spacecraft attitude with Runge-Kutta method
 */
#include "attitude_rk4.hpp"

#include <library/logger/log_utility.hpp>
#include <library/utilities/macros.hpp>
using namespace std;

#include <iostream>
#include <sstream>

AttitudeRK4::AttitudeRK4(const Vector<3>& omega_b_ini, const Quaternion& quaternion_i2b_ini, const Matrix<3, 3>& InertiaTensor_ini,
                         const Vector<3>& torque_b_ini, const double prop_step_ini, const std::string& sim_object_name)
    : Attitude(sim_object_name) {
  angular_velocity_b_rad_s_ = omega_b_ini;
  quaternion_i2b_ = quaternion_i2b_ini;
  torque_b_Nm_ = torque_b_ini;
  inertia_tensor_kgm2_ = InertiaTensor_ini;
  propagation_step_s_ = prop_step_ini;
  current_propagation_time_s_ = 0.0;
  inv_inertia_tensor_ = invert(inertia_tensor_kgm2_);
  angular_momentum_reaction_wheel_b_Nms_ = libra::Vector<3>(0.0);
  CalcAngularMomentum();
}

AttitudeRK4::~AttitudeRK4() {}

void AttitudeRK4::SetParameters(const MCSimExecutor& mc_sim) {
  Attitude::SetParameters(mc_sim);
  GetInitParameterVec(mc_sim, "Omega_b", angular_velocity_b_rad_s_);

  // TODO: Consider the following calculation is needed here?
  current_propagation_time_s_ = 0.0;
  inv_inertia_tensor_ = libra::invert(inertia_tensor_kgm2_);
  angular_momentum_reaction_wheel_b_Nms_ = Vector<3>(0.0);  //!< Consider how to handle this variable
  CalcAngularMomentum();
}

void AttitudeRK4::Propagate(const double endtime_s) {
  if (!is_calc_enabled_) return;
  while (endtime_s - current_propagation_time_s_ - propagation_step_s_ > 1.0e-6) {
    RungeKuttaOneStep(current_propagation_time_s_, propagation_step_s_);
    current_propagation_time_s_ += propagation_step_s_;
  }
  RungeKuttaOneStep(current_propagation_time_s_, endtime_s - current_propagation_time_s_);
  current_propagation_time_s_ = endtime_s;

  CalcAngularMomentum();
}

Matrix<4, 4> AttitudeRK4::CalcAngularVelocityMatrix(Vector<3> angular_velocity_b_rad_s) {
  Matrix<4, 4> angular_velocity_matrix;

  angular_velocity_matrix[0][0] = 0.0f;
  angular_velocity_matrix[0][1] = angular_velocity_b_rad_s[2];
  angular_velocity_matrix[0][2] = -angular_velocity_b_rad_s[1];
  angular_velocity_matrix[0][3] = angular_velocity_b_rad_s[0];
  angular_velocity_matrix[1][0] = -angular_velocity_b_rad_s[2];
  angular_velocity_matrix[1][1] = 0.0f;
  angular_velocity_matrix[1][2] = angular_velocity_b_rad_s[0];
  angular_velocity_matrix[1][3] = angular_velocity_b_rad_s[1];
  angular_velocity_matrix[2][0] = angular_velocity_b_rad_s[1];
  angular_velocity_matrix[2][1] = -angular_velocity_b_rad_s[0];
  angular_velocity_matrix[2][2] = 0.0f;
  angular_velocity_matrix[2][3] = angular_velocity_b_rad_s[2];
  angular_velocity_matrix[3][0] = -angular_velocity_b_rad_s[0];
  angular_velocity_matrix[3][1] = -angular_velocity_b_rad_s[1];
  angular_velocity_matrix[3][2] = -angular_velocity_b_rad_s[2];
  angular_velocity_matrix[3][3] = 0.0f;

  return angular_velocity_matrix;
}

Vector<7> AttitudeRK4::AttitudeDynamicsAndKinematics(Vector<7> x, double t) {
  UNUSED(t);

  Vector<7> dxdt;

  Vector<3> omega_b;
  for (int i = 0; i < 3; i++) {
    omega_b[i] = x[i];
  }
  angular_momentum_total_b_Nms_ = (inertia_tensor_kgm2_ * omega_b) + angular_momentum_reaction_wheel_b_Nms_;
  Vector<3> rhs = inv_inertia_tensor_ * (torque_b_Nm_ - libra::outer_product(omega_b, angular_momentum_total_b_Nms_));

  for (int i = 0; i < 3; ++i) {
    dxdt[i] = rhs[i];
  }

  Vector<4> quaternion_i2b;
  for (int i = 0; i < 4; i++) {
    quaternion_i2b[i] = x[i + 3];
  }

  Vector<4> d_quaternion = 0.5 * CalcAngularVelocityMatrix(omega_b) * quaternion_i2b;

  for (int i = 0; i < 4; i++) {
    dxdt[i + 3] = d_quaternion[i];
  }

  return dxdt;
}

void AttitudeRK4::RungeKuttaOneStep(double t, double dt) {
  Vector<7> x;
  for (int i = 0; i < 3; i++) {
    x[i] = angular_velocity_b_rad_s_[i];
  }
  for (int i = 0; i < 4; i++) {
    x[i + 3] = quaternion_i2b_[i];
  }

  Vector<7> k1, k2, k3, k4;
  Vector<7> xk2, xk3, xk4;

  k1 = AttitudeDynamicsAndKinematics(x, t);
  xk2 = x + (dt / 2.0) * k1;

  k2 = AttitudeDynamicsAndKinematics(xk2, (t + dt / 2.0));
  xk3 = x + (dt / 2.0) * k2;

  k3 = AttitudeDynamicsAndKinematics(xk3, (t + dt / 2.0));
  xk4 = x + dt * k3;

  k4 = AttitudeDynamicsAndKinematics(xk4, (t + dt));

  Vector<7> next_x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

  for (int i = 0; i < 3; i++) {
    angular_velocity_b_rad_s_[i] = next_x[i];
  }
  for (int i = 0; i < 4; i++) {
    quaternion_i2b_[i] = next_x[i + 3];
  }
  quaternion_i2b_.normalize();
}
