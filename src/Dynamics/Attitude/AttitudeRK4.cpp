#include "AttitudeRK4.h"

#include <Interface/LogOutput/LogUtility.h>
using namespace std;

#include <iostream>
#include <sstream>

AttitudeRK4::AttitudeRK4(const Vector<3>& omega_b_ini,
                         const Quaternion& quaternion_i2b_ini,
                         const Matrix<3, 3>& InertiaTensor_ini,
                         const Vector<3>& torque_b_ini,
                         const double prop_step_ini)
    : SimulationObject("Attitude") {
  omega_b_ = omega_b_ini;
  quaternion_i2b_ = quaternion_i2b_ini;
  torque_b_ = torque_b_ini;
  inertia_tensor_ = InertiaTensor_ini;
  prop_step_ = prop_step_ini;
  prop_time_ = 0;
  inv_inertia_tensor_ = invert(inertia_tensor_);
  h_rw_b_ = Vector<3>(
      0);  //どう取り扱うか要検討，Propagateで参照しているのも良くないかも
  CalcAngMom();
}

AttitudeRK4::AttitudeRK4(const Vector<3>& omega_b_ini,
                         const Quaternion& quaternion_i2b_ini,
                         const Matrix<3, 3>& InertiaTensor_ini,
                         const Vector<3>& torque_b_ini,
                         const double prop_step_ini, string name)
    : SimulationObject(name) {
  omega_b_ = omega_b_ini;
  quaternion_i2b_ = quaternion_i2b_ini;
  torque_b_ = torque_b_ini;
  inertia_tensor_ = InertiaTensor_ini;
  prop_step_ = prop_step_ini;
  prop_time_ = 0;
  inv_inertia_tensor_ = invert(inertia_tensor_);
  h_rw_b_ = Vector<3>(
      0);  //どう取り扱うか要検討，Propagateで参照しているのも良くないかも
  CalcAngMom();
  CalcSatRotationalKineticEnergy();
}

AttitudeRK4::~AttitudeRK4() {}

void AttitudeRK4::SetParameters(const MCSimExecutor& mc_sim) {
  GetInitParameterVec(mc_sim, "Debug", debug_vec_);
  // cout << "Attitude.Debug = " << debug_vec_[0] << ", " << debug_vec_[1] << ",
  // " << debug_vec_[2] << endl;
  GetInitParameterVec(mc_sim, "Omega_b", omega_b_);
  GetInitParameterQuaternion(mc_sim, "Q_i2b", quaternion_i2b_);
  // cout << "Attitude.Omega_b = " << omega_b_[0] << ", " << omega_b_[1] << ", "
  // << omega_b_[2] << endl;
  prop_time_ = 0;
  inv_inertia_tensor_ = invert(inertia_tensor_);
  h_rw_b_ = Vector<3>(
      0);  //どう取り扱うか要検討，Propagateで参照しているのも良くないかも
  CalcAngMom();
  CalcSatRotationalKineticEnergy();
}

void AttitudeRK4::CalcAngMom(void) {
  h_sc_b_ = inertia_tensor_ * omega_b_;
  h_total_b_ = h_rw_b_ + h_sc_b_;
  Quaternion q_b2i = quaternion_i2b_.conjugate();
  h_total_i_ = q_b2i.frame_conv(h_total_b_);
  h_total_ = norm(h_total_i_);
}

void AttitudeRK4::CalcSatRotationalKineticEnergy(void) {
  k_sc_ = 0.5 * libra::inner_product(h_sc_b_, omega_b_);
}

void AttitudeRK4::Propagate(double endtime) {
  if (!IsCalcEnabled) return;
  while (endtime - prop_time_ - prop_step_ > 1.0e-6) {
    RungeOneStep(prop_time_, prop_step_);
    prop_time_ += prop_step_;
  }
  RungeOneStep(prop_time_, endtime - prop_time_);
  prop_time_ = endtime;

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
  Vector<7> dxdt;

  Vector<3> omega_b;
  for (int i = 0; i < 3; i++) {
    omega_b[i] = x[i];
  }
  h_total_b_ = (inertia_tensor_ * omega_b) + h_rw_b_;
  Vector<3> rhs =
      inv_inertia_tensor_ * (torque_b_ - outer_product(omega_b, h_total_b_));

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
    x[i] = omega_b_[i];
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
    omega_b_[i] = next_x[i];
  }
  for (int i = 0; i < 4; i++) {
    quaternion_i2b_[i] = next_x[i + 3];
  }
  quaternion_i2b_.normalize();
}

string AttitudeRK4::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("omega_t", "b", "rad/s", 3);
  str_tmp += WriteVector("q_t", "i2b", "-", 4);
  str_tmp += WriteVector("torque_t", "b", "Nm", 3);
  // str_tmp += WriteVector("h_total", "b", "Nms", 3);
  // str_tmp += WriteVector("h_total", "i", "Nms", 3);
  str_tmp += WriteScalar("h_total", "Nms");
  str_tmp += WriteScalar("k_sc", "J");

  return str_tmp;
}

string AttitudeRK4::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(omega_b_);
  str_tmp += WriteQuaternion(quaternion_i2b_);
  str_tmp += WriteVector(torque_b_);
  // str_tmp += WriteVector(h_total_b_);
  // str_tmp += WriteVector(h_total_i_);
  str_tmp += WriteScalar(h_total_);
  str_tmp += WriteScalar(k_sc_);

  return str_tmp;
}

//デバッグ用
void AttitudeRK4::PrintParams(void) {
  cout << "Omega_b =(" << omega_b_[0] << "," << omega_b_[1] << ","
       << omega_b_[2] << ") rad/s \n";
  cout << "Quaternion_i2b =(" << quaternion_i2b_[0] << "," << quaternion_i2b_[1]
       << "," << quaternion_i2b_[2] << "," << quaternion_i2b_[3] << ") \n";
  cout << "Torque_b =(" << torque_b_[0] << "," << torque_b_[1] << ","
       << torque_b_[2] << ") Nm \n";
  cout << "               (" << inertia_tensor_[0][0] << ","
       << inertia_tensor_[0][1] << "," << inertia_tensor_[0][2] << ")\n";
  cout << "InertiaTensor =(" << inertia_tensor_[1][0] << ","
       << inertia_tensor_[1][1] << "," << inertia_tensor_[1][2] << ") kg m2\n";
  cout << "               (" << inertia_tensor_[2][0] << ","
       << inertia_tensor_[2][1] << "," << inertia_tensor_[2][2] << ")\n";
}
