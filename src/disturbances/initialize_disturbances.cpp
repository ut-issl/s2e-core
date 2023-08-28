/**
 * @file initialize_disturbances.cpp
 * @brief Define initialize functions for disturbances
 */

#include "initialize_disturbances.hpp"

#include <library/initialize/initialize_file_access.hpp>

#define CALC_LABEL "calculation"
#define LOG_LABEL "logging"

GravityGradient InitGravityGradient(const std::string initialize_file_path) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "GRAVITY_GRADIENT";

  const bool is_calc_enable = conf.ReadEnable(section, CALC_LABEL);
  GravityGradient gg_disturbance(is_calc_enable);
  gg_disturbance.is_log_enabled_ = conf.ReadEnable(section, LOG_LABEL);

  return gg_disturbance;
}

GravityGradient InitGravityGradient(const std::string initialize_file_path, const double gravity_constant_m3_s2) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "GRAVITY_GRADIENT";

  const bool is_calc_enable = conf.ReadEnable(section, CALC_LABEL);
  GravityGradient gg_disturbance(gravity_constant_m3_s2, is_calc_enable);
  gg_disturbance.is_log_enabled_ = conf.ReadEnable(section, LOG_LABEL);

  return gg_disturbance;
}

MagneticDisturbance InitMagneticDisturbance(const std::string initialize_file_path, const ResidualMagneticMoment& rmm_params) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "MAGNETIC_DISTURBANCE";

  const bool is_calc_enable = conf.ReadEnable(section, CALC_LABEL);
  MagneticDisturbance mag_disturbance(rmm_params, is_calc_enable);
  mag_disturbance.is_log_enabled_ = conf.ReadEnable(section, LOG_LABEL);

  return mag_disturbance;
}

Geopotential InitGeopotential(const std::string initialize_file_path) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "GEOPOTENTIAL";

  const int degree = conf.ReadInt(section, "degree");
  const std::string coefficients_file_path = conf.ReadString(section, "coefficients_file_path");

  const bool is_calc_enable = conf.ReadEnable(section, CALC_LABEL);
  Geopotential geopotential_disturbance(degree, coefficients_file_path, is_calc_enable);
  geopotential_disturbance.is_log_enabled_ = conf.ReadEnable(section, LOG_LABEL);

  return geopotential_disturbance;
}

LunarGravityField InitLunarGravityField(const std::string initialize_file_path) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "LUNAR_GRAVITY_FIELD";

  const int degree = conf.ReadInt(section, "degree");
  const std::string coefficients_file_path = conf.ReadString(section, "coefficients_file_path");

  const bool is_calc_enable = conf.ReadEnable(section, CALC_LABEL);

  LunarGravityField lunar_gravity_field(degree, coefficients_file_path, is_calc_enable);
  lunar_gravity_field.is_log_enabled_ = conf.ReadEnable(section, LOG_LABEL);

  return lunar_gravity_field;
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
  if (conf.ReadEnable(section, CALC_LABEL)) {
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

  const bool is_calc_enable = conf.ReadEnable(section, CALC_LABEL);
  ThirdBodyGravity third_body_disturbance(third_body_list, is_calc_enable);
  third_body_disturbance.is_log_enabled_ = conf.ReadEnable(section, LOG_LABEL);

  return third_body_disturbance;
}
