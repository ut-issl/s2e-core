/**
 * @file attitude.cpp
 * @brief Base class for attitude of spacecraft
 */
#include "attitude.hpp"

#include <library/logger/log_utility.hpp>

Attitude::Attitude(const std::string& sim_object_name) : SimulationObject(sim_object_name) {
  omega_b_rad_s_ = libra::Vector<3>(0.0);
  quaternion_i2b_ = libra::Quaternion(0.0, 0.0, 0.0, 1.0);
  torque_b_Nm_ = libra::Vector<3>(0.0);
  h_sc_b_Nms_ = libra::Vector<3>(0.0);
  h_rw_b_Nms_ = libra::Vector<3>(0.0);
  h_total_b_Nms_ = libra::Vector<3>(0.0);
  h_total_i_Nms_ = libra::Vector<3>(0.0);
  h_total_Nms_ = 0.0;
  k_sc_J_ = 0.0;
}

std::string Attitude::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("spacecraft_angular_velocity", "b", "rad/s", 3);
  str_tmp += WriteQuaternion("spacecraft_quaternion", "i2b");
  str_tmp += WriteVector("spacecraft_torque", "b", "Nm", 3);
  str_tmp += WriteScalar("spacecraft_total_angular_momentum", "Nms");
  str_tmp += WriteScalar("spacecraft_kinematic_energy", "J");

  return str_tmp;
}

std::string Attitude::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(omega_b_rad_s_);
  str_tmp += WriteQuaternion(quaternion_i2b_);
  str_tmp += WriteVector(torque_b_Nm_);
  str_tmp += WriteScalar(h_total_Nms_);
  str_tmp += WriteScalar(k_sc_J_);

  return str_tmp;
}

void Attitude::SetParameters(const MCSimExecutor& mc_sim) { GetInitParameterQuaternion(mc_sim, "Q_i2b", quaternion_i2b_); }

void Attitude::CalcAngMom(void) {
  h_sc_b_Nms_ = inertia_tensor_kgm2_ * omega_b_rad_s_;
  h_total_b_Nms_ = h_rw_b_Nms_ + h_sc_b_Nms_;
  Quaternion q_b2i = quaternion_i2b_.conjugate();
  h_total_i_Nms_ = q_b2i.frame_conv(h_total_b_Nms_);
  h_total_Nms_ = norm(h_total_i_Nms_);
}

void Attitude::CalcSatRotationalKineticEnergy(void) { k_sc_J_ = 0.5 * libra::inner_product(h_sc_b_Nms_, omega_b_rad_s_); }
