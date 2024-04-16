/**
 * @file attitude_rk4.cpp
 * @brief Class to calculate spacecraft attitude with Runge-Kutta method
 */
#include "attitude_rk4.hpp"

#include <iostream>
#include <logger/log_utility.hpp>
#include <sstream>
#include <utilities/macros.hpp>

AttitudeRk4::AttitudeRk4(const libra::Vector<3>& angular_velocity_b_rad_s, const libra::Quaternion& quaternion_i2b,
                         const libra::Matrix<3, 3>& inertia_tensor_kgm2, const libra::Vector<3>& torque_b_Nm, const double propagation_step_s,
                         const std::string& simulation_object_name)
    : Attitude(inertia_tensor_kgm2, simulation_object_name) {
  angular_velocity_b_rad_s_ = angular_velocity_b_rad_s;
  quaternion_i2b_ = quaternion_i2b;
  torque_b_Nm_ = torque_b_Nm;
  propagation_step_s_ = propagation_step_s;
  current_propagation_time_s_ = 0.0;
  angular_momentum_reaction_wheel_b_Nms_ = libra::Vector<3>(0.0);
  previous_inertia_tensor_kgm2_ = inertia_tensor_kgm2_;
  inverse_inertia_tensor_ = CalcInverseMatrix(inertia_tensor_kgm2_);
  CalcAngularMomentum();
}

AttitudeRk4::AttitudeRk4(const libra::Vector<3>& angular_velocity_b_rad_s, const libra::Quaternion& quaternion_i2b,
                         const libra::Matrix<3, 3>& inertia_tensor_kgm2, const libra::Matrix<3, 3>& inertia_tensor_flexible_kgm2,
                         const double zeta_flexible, const double omega_flexible_rad_s, const libra::Vector<3>& torque_b_Nm,
                         const double propagation_step_s, const std::string& simulation_object_name)
    : Attitude(inertia_tensor_kgm2, simulation_object_name) {
  angular_velocity_b_rad_s_ = angular_velocity_b_rad_s;
  quaternion_i2b_ = quaternion_i2b;
  torque_b_Nm_ = torque_b_Nm;
  propagation_step_s_ = propagation_step_s;
  current_propagation_time_s_ = 0.0;
  angular_momentum_reaction_wheel_b_Nms_ = libra::Vector<3>(0.0);
  previous_inertia_tensor_kgm2_ = inertia_tensor_kgm2_;
  inertia_tensor_flexible_kgm2_ = inertia_tensor_flexible_kgm2;
  zeta_flexible_ = zeta_flexible;
  omega_flexible_rad_s_ = omega_flexible_rad_s;
  inverse_inertia_tensor_ = CalcInverseMatrix(inertia_tensor_kgm2_);
  inverse_equivalent_inertia_tensor_flexible_ = CalcInverseMatrix(inertia_tensor_kgm2_) * (inertia_tensor_kgm2_ + inertia_tensor_flexible_kgm2_) *
                                                CalcInverseMatrix(inertia_tensor_flexible_kgm2_);
  inverse_inertia_tensor_total_ = CalcInverseMatrix(inertia_tensor_kgm2_ + inertia_tensor_flexible_kgm2_);
  CalcAngularMomentum();
}

AttitudeRk4::~AttitudeRk4() {}

std::string AttitudeRk4::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("eular_angular_flexible", "c", "rad", 3);
  str_tmp += WriteVector("angular_velocity_flexible", "c", "rad/s", 3);

  return str_tmp;
}

std::string AttitudeRk4::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(eular_angular_flexible_rad_);
  str_tmp += WriteVector(angular_velocity_flexible_rad_s_);

  return str_tmp;
}

void AttitudeRk4::SetParameters(const MonteCarloSimulationExecutor& mc_simulator) {
  Attitude::SetParameters(mc_simulator);
  GetInitializedMonteCarloParameterVector(mc_simulator, "angular_velocity_b_rad_s", angular_velocity_b_rad_s_);

  // TODO: Consider the following calculation is needed here?
  current_propagation_time_s_ = 0.0;
  angular_momentum_reaction_wheel_b_Nms_ = libra::Vector<3>(0.0);  //!< Consider how to handle this variable
  CalcAngularMomentum();
}

void AttitudeRk4::Propagate(const double end_time_s) {
  if (!is_calc_enabled_) return;

  libra::Matrix<3, 3> dot_inertia_tensor =
      (1.0 / (end_time_s - current_propagation_time_s_)) * (inertia_tensor_kgm2_ - previous_inertia_tensor_kgm2_);
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

libra::Matrix<4, 4> AttitudeRk4::CalcAngularVelocityMatrix(libra::Vector<3> angular_velocity_b_rad_s) {
  libra::Matrix<4, 4> angular_velocity_matrix;

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

libra::Vector<10> AttitudeRk4::AttitudeDynamicsAndKinematics(libra::Vector<10> x, double t) {
  UNUSED(t);

  libra::Vector<10> dxdt;

  libra::Vector<3> omega_b;
  for (int i = 0; i < 3; i++) {
    omega_b[i] = x[i];
  }
  libra::Vector<3> omega_flexible;
  for (int i = 0; i < 3; i++) {
    omega_flexible[i] = x[i + 7];
  }

  libra::Vector<3> angular_momentum_total_b_Nms = (previous_inertia_tensor_kgm2_ * omega_b) + angular_momentum_reaction_wheel_b_Nms_;
  libra::Vector<3> net_torque_b_Nm = torque_b_Nm_ - libra::OuterProduct(omega_b, angular_momentum_total_b_Nms) - torque_inertia_tensor_change_b_Nm_;

  libra::Vector<3> angular_accelaration_flexible_rad_s2 =
      inverse_equivalent_inertia_tensor_flexible_ *
          (-2 * zeta_flexible_ * omega_flexible_rad_s_ * inertia_tensor_flexible_kgm2_ * omega_flexible -
           omega_flexible_rad_s_ * omega_flexible_rad_s_ * inertia_tensor_flexible_kgm2_ * eular_angular_flexible_rad_) -
      inverse_inertia_tensor_ * net_torque_b_Nm;

  libra::Vector<3> rhs = inverse_inertia_tensor_total_ * net_torque_b_Nm - inertia_tensor_flexible_kgm2_ * angular_accelaration_flexible_rad_s2;

  for (int i = 0; i < 3; ++i) {
    dxdt[i] = rhs[i];
  }

  libra::Vector<4> quaternion_i2b;
  for (int i = 0; i < 4; i++) {
    quaternion_i2b[i] = x[i + 3];
  }

  libra::Vector<4> d_quaternion = 0.5 * CalcAngularVelocityMatrix(omega_b) * quaternion_i2b;

  for (int i = 0; i < 4; i++) {
    dxdt[i + 3] = d_quaternion[i];
  }

  for (int i = 0; i < 3; i++) {
    dxdt[i + 7] = angular_accelaration_flexible_rad_s2[i];
  }

  return dxdt;
}

void AttitudeRk4::RungeKuttaOneStep(double t, double dt) {
  libra::Vector<10> x;
  for (int i = 0; i < 3; i++) {
    x[i] = angular_velocity_b_rad_s_[i];
  }
  for (int i = 0; i < 4; i++) {
    x[i + 3] = quaternion_i2b_[i];
  }
  for (int i = 0; i < 3; i++) {
    x[i + 7] = angular_velocity_flexible_rad_s_[i];
  }

  libra::Vector<10> k1, k2, k3, k4;
  libra::Vector<10> xk2, xk3, xk4;

  k1 = AttitudeDynamicsAndKinematics(x, t);
  xk2 = x + (dt / 2.0) * k1;

  k2 = AttitudeDynamicsAndKinematics(xk2, (t + dt / 2.0));
  xk3 = x + (dt / 2.0) * k2;

  k3 = AttitudeDynamicsAndKinematics(xk3, (t + dt / 2.0));
  xk4 = x + dt * k3;

  k4 = AttitudeDynamicsAndKinematics(xk4, (t + dt));

  libra::Vector<10> next_x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

  for (int i = 0; i < 3; i++) {
    angular_velocity_b_rad_s_[i] = next_x[i];
  }
  for (int i = 0; i < 4; i++) {
    quaternion_i2b_[i] = next_x[i + 3];
  }
  quaternion_i2b_.Normalize();

  eular_angular_flexible_rad_ += dt * angular_velocity_flexible_rad_s_;
  for (int i = 0; i < 3; i++) {
    angular_velocity_flexible_rad_s_[i] = next_x[i + 7];
  }
}
