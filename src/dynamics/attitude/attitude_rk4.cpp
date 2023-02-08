/**
 * @file attitude_rk4.cpp
 * @brief Class to calculate spacecraft attitude with Runge-Kutta method
 */
#include "attitude_rk4.hpp"

#include <interface/log_output/log_utility.hpp>

#include <Library/utils/Macros.hpp>
using namespace std;

#include <iostream>
#include <sstream>

AttitudeRK4::AttitudeRK4(const Vector<3>& omega_b_ini, const Quaternion& quaternion_i2b_ini, const Matrix<3, 3>& InertiaTensor_ini,
                         const Vector<3>& torque_b_ini, const double prop_step_ini, const std::string& sim_object_name)
    : Attitude(sim_object_name) {
  omega_b_rad_s_ = omega_b_ini;
  quaternion_i2b_ = quaternion_i2b_ini;
  torque_b_Nm_ = torque_b_ini;
  inertia_tensor_kgm2_ = InertiaTensor_ini;
  prop_step_s_ = prop_step_ini;
  prop_time_s_ = 0.0;
  inv_inertia_tensor_ = invert(inertia_tensor_kgm2_);
  h_rw_b_Nms_ = libra::Vector<3>(0.0);
  CalcAngMom();
}

AttitudeRK4::~AttitudeRK4() {}

void AttitudeRK4::SetParameters(const MCSimExecutor& mc_sim) {
  Attitude::SetParameters(mc_sim);
  GetInitParameterVec(mc_sim, "Omega_b", omega_b_rad_s_);

  // TODO: Consider the following calculation is needed here?
  prop_time_s_ = 0.0;
  inv_inertia_tensor_ = libra::invert(inertia_tensor_kgm2_);
  h_rw_b_Nms_ = Vector<3>(0.0);  //!< Consider how to handle this variable
  CalcAngMom();
  CalcSatRotationalKineticEnergy();
}

void AttitudeRK4::Propagate(const double endtime_s) {
  if (!is_calc_enabled_) return;
  while (endtime_s - prop_time_s_ - prop_step_s_ > 1.0e-6) {
    RungeOneStep(prop_time_s_, prop_step_s_);
    prop_time_s_ += prop_step_s_;
  }
  RungeOneStep(prop_time_s_, endtime_s - prop_time_s_);
  prop_time_s_ = endtime_s;

  CalcAngMom();
  CalcSatRotationalKineticEnergy();
}

Matrix<4, 4> AttitudeRK4::Omega4Kinematics(Vector<3> omega) {
  Matrix<4, 4> OMEGA;

  OMEGA[0][0] = 0.0f;
  OMEGA[0][1] = omega[2];
  OMEGA[0][2] = -omega[1];
  OMEGA[0][3] = omega[0];
  OMEGA[1][0] = -omega[2];
  OMEGA[1][1] = 0.0f;
  OMEGA[1][2] = omega[0];
  OMEGA[1][3] = omega[1];
  OMEGA[2][0] = omega[1];
  OMEGA[2][1] = -omega[0];
  OMEGA[2][2] = 0.0f;
  OMEGA[2][3] = omega[2];
  OMEGA[3][0] = -omega[0];
  OMEGA[3][1] = -omega[1];
  OMEGA[3][2] = -omega[2];
  OMEGA[3][3] = 0.0f;

  return OMEGA;
}

Vector<7> AttitudeRK4::DynamicsKinematics(Vector<7> x, double t) {
  UNUSED(t);

  Vector<7> dxdt;

  Vector<3> omega_b;
  for (int i = 0; i < 3; i++) {
    omega_b[i] = x[i];
  }
  h_total_b_Nms_ = (inertia_tensor_kgm2_ * omega_b) + h_rw_b_Nms_;
  Vector<3> rhs = inv_inertia_tensor_ * (torque_b_Nm_ - libra::outer_product(omega_b, h_total_b_Nms_));

  for (int i = 0; i < 3; ++i) {
    dxdt[i] = rhs[i];
  }

  Vector<4> quaternion_i2b;
  for (int i = 0; i < 4; i++) {
    quaternion_i2b[i] = x[i + 3];
  }

  Vector<4> d_quaternion = 0.5 * Omega4Kinematics(omega_b) * quaternion_i2b;

  for (int i = 0; i < 4; i++) {
    dxdt[i + 3] = d_quaternion[i];
  }

  return dxdt;
}

void AttitudeRK4::RungeOneStep(double t, double dt) {
  Vector<7> x;
  for (int i = 0; i < 3; i++) {
    x[i] = omega_b_rad_s_[i];
  }
  for (int i = 0; i < 4; i++) {
    x[i + 3] = quaternion_i2b_[i];
  }

  Vector<7> k1, k2, k3, k4;
  Vector<7> xk2, xk3, xk4;

  k1 = DynamicsKinematics(x, t);
  xk2 = x + (dt / 2.0) * k1;

  k2 = DynamicsKinematics(xk2, (t + dt / 2.0));
  xk3 = x + (dt / 2.0) * k2;

  k3 = DynamicsKinematics(xk3, (t + dt / 2.0));
  xk4 = x + dt * k3;

  k4 = DynamicsKinematics(xk4, (t + dt));

  Vector<7> next_x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

  for (int i = 0; i < 3; i++) {
    omega_b_rad_s_[i] = next_x[i];
  }
  for (int i = 0; i < 4; i++) {
    quaternion_i2b_[i] = next_x[i + 3];
  }
  quaternion_i2b_.normalize();
}
