/**
 * @file third_body_gravity.cpp
 * @brief Class to calculate third body gravity disturbance
 */

#include "third_body_gravity.hpp"

#include <setting_file_reader/initialize_file_access.hpp>

ThirdBodyGravity::ThirdBodyGravity(std::set<std::string> third_body_list, const bool is_calculation_enabled)
    : Disturbance(is_calculation_enabled, false), third_body_list_(third_body_list) {
  acceleration_i_m_s2_ = s2e::math::Vector<3>(0.0);
}

ThirdBodyGravity::~ThirdBodyGravity() {}

void ThirdBodyGravity::Update(const LocalEnvironment& local_environment, const Dynamics& dynamics) {
  acceleration_i_m_s2_ = s2e::math::Vector<3>(0.0);  // initialize

  s2e::math::Vector<3> sc_position_i_m = dynamics.GetOrbit().GetPosition_i_m();
  for (auto third_body : third_body_list_) {
    s2e::math::Vector<3> third_body_position_from_sc_i_m = local_environment.GetCelestialInformation().GetPositionFromSpacecraft_i_m(third_body.c_str());
    s2e::math::Vector<3> third_body_pos_i_m = sc_position_i_m + third_body_position_from_sc_i_m;
    double gravity_constant = local_environment.GetCelestialInformation().GetGlobalInformation().GetGravityConstant_m3_s2(third_body.c_str());

    third_body_acceleration_i_m_s2_ = CalcAcceleration_i_m_s2(third_body_pos_i_m, third_body_position_from_sc_i_m, gravity_constant);
    acceleration_i_m_s2_ += third_body_acceleration_i_m_s2_;
  }
}

s2e::math::Vector<3> ThirdBodyGravity::CalcAcceleration_i_m_s2(const s2e::math::Vector<3> s, const s2e::math::Vector<3> sr, const double gravity_constant_m_s2) {
  s2e::math::Vector<3> acceleration_i_m_s2;

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

ThirdBodyGravity InitThirdBodyGravity(const std::string initialize_file_path, const std::string ini_path_celes) {
  // Generate a list of bodies to be calculated in "CelesInfo"
  auto conf_celes = IniAccess(ini_path_celes);
  const char* section_celes = "CELESTIAL_INFORMATION";
  const int num_of_selected_body = conf_celes.ReadInt(section_celes, "number_of_selected_body");
  const std::string center_object = conf_celes.ReadString(section_celes, "center_object");

  std::set<std::string> selected_body_list;
  for (int i = 0; i < num_of_selected_body; i++) {
    std::string selected_body_id = "selected_body_name(" + std::to_string(i) + ")";
    selected_body_list.insert(conf_celes.ReadString(section_celes, selected_body_id.c_str()));
  }

  // Generate a list of bodies to be calculated in "ThirdBodyGravity" from the list of bodies of "CelesInfo"
  auto conf = IniAccess(initialize_file_path);
  const char* section = "THIRD_BODY_GRAVITY";

  const int num_of_third_body = conf.ReadInt(section, "number_of_third_body");

  std::set<std::string> third_body_list;
  // Generate the list of the third object if "calculation=ENABLE"
  if (conf.ReadEnable(section, INI_CALC_LABEL)) {
    for (int i = 0; i < num_of_third_body; i++) {
      const std::string third_body_id = "third_body_name(" + std::to_string(i) + ")";
      const std::string third_body_name = conf.ReadString(section, third_body_id.c_str());
      // If the object specified by `third_body` in "SampleDisturbance.ini" is
      // the center object of the orbital propagation, the system prints an
      // error message.
      assert(third_body_name != center_object);
      // If the target specified by `third_body` in "SampleDisturbance.ini" is
      // not in the list of bodies to be calculated by "CelesInfo", the system
      // prints an error message.
      assert(selected_body_list.find(third_body_name) != selected_body_list.end());
      third_body_list.insert(third_body_name);
    }
  }

  const bool is_calc_enable = conf.ReadEnable(section, INI_CALC_LABEL);
  ThirdBodyGravity third_body_disturbance(third_body_list, is_calc_enable);
  third_body_disturbance.is_log_enabled_ = conf.ReadEnable(section, INI_LOG_LABEL);

  return third_body_disturbance;
}
