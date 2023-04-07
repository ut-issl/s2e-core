/**
 * @file third_body_gravity.cpp
 * @brief Class to calculate third body gravity disturbance
 */

#include "third_body_gravity.hpp"

ThirdBodyGravity::ThirdBodyGravity(std::set<std::string> third_body_list, const bool is_calculation_enabled)
    : Disturbance(is_calculation_enabled, false), third_body_list_(third_body_list) {
  acceleration_i_m_s2_ = libra::Vector<3>(0.0);
}

ThirdBodyGravity::~ThirdBodyGravity() {}

void ThirdBodyGravity::Update(const LocalEnvironment& local_environment, const Dynamics& dynamics) {
  acceleration_i_m_s2_ = libra::Vector<3>(0.0);  // initialize

  libra::Vector<3> sc_position_i_m = dynamics.GetOrbit().GetPosition_i_m();
  for (auto third_body : third_body_list_) {
    libra::Vector<3> third_body_position_from_sc_i_m = local_environment.GetCelestialInformation().GetPositionFromSpacecraft_i_m(third_body.c_str());
    libra::Vector<3> third_body_pos_i_m = sc_position_i_m + third_body_position_from_sc_i_m;
    double gravity_constant = local_environment.GetCelestialInformation().GetGlobalInformation().GetGravityConstant_m3_s2(third_body.c_str());

    third_body_acceleration_i_m_s2_ = CalcAcceleration_i_m_s2(third_body_pos_i_m, third_body_position_from_sc_i_m, gravity_constant);
    acceleration_i_m_s2_ += third_body_acceleration_i_m_s2_;
  }
}

libra::Vector<3> ThirdBodyGravity::CalcAcceleration_i_m_s2(const libra::Vector<3> s, const libra::Vector<3> sr, const double gravity_constant_m_s2) {
  libra::Vector<3> acceleration_i_m_s2;

  double s_norm = s.CalcNorm();
  double s_norm3 = s_norm * s_norm * s_norm;

  double sr_norm = sr.CalcNorm();
  double sr_norm3 = sr_norm * sr_norm * sr_norm;

  acceleration_i_m_s2 = gravity_constant_m_s2 * (1.0 / sr_norm3 * sr - 1.0 / s_norm3 * s);

  return acceleration_i_m_s2;
}

std::string ThirdBodyGravity::GetLogHeader() const {
  std::string str_tmp = "";
  str_tmp += WriteVector("third_body_acceleration", "i", "m/s2", 3);

  return str_tmp;
}

std::string ThirdBodyGravity::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteVector(acceleration_i_m_s2_);

  return str_tmp;
}
