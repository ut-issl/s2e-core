/**
 * @file InitDisturbance.cpp
 * @brief Define initialize functions for disturbances
 */

#include "InitDisturbance.hpp"

#include <Interface/InitInput/IniAccess.h>

#define CALC_LABEL "calculation"
#define LOG_LABEL "logging"
#define MIN_VAL 1e-9

AirDrag InitAirDrag(std::string ini_path, const std::vector<Surface>& surfaces, const Vector<3>& cg_b) {
  auto conf = IniAccess(ini_path);
  const char* section = "AIR_DRAG";

  double t_w = conf.ReadDouble(section, "wall_temperature_degC") + 273.0;
  double t_m = conf.ReadDouble(section, "molecular_temperature_degC") + 273.0;
  double molecular = conf.ReadDouble(section, "molecular_weight");

  bool calcen = conf.ReadEnable(section, CALC_LABEL);
  bool logen = conf.ReadEnable(section, LOG_LABEL);

  AirDrag airdrag(surfaces, cg_b, t_w, t_m, molecular);
  airdrag.IsCalcEnabled = calcen;
  airdrag.IsLogEnabled = logen;

  return airdrag;
}

SolarRadiation InitSRDist(std::string ini_path, const std::vector<Surface>& surfaces, const Vector<3>& cg_b) {
  auto conf = IniAccess(ini_path);
  const char* section = "SOLAR_RADIATION_PRESSURE";

  bool calcen = conf.ReadEnable(section, CALC_LABEL);
  bool logen = conf.ReadEnable(section, LOG_LABEL);

  SolarRadiation srdist(surfaces, cg_b);
  srdist.IsCalcEnabled = calcen;
  srdist.IsLogEnabled = logen;

  return srdist;
}

GravityGradient InitGravityGradient(std::string ini_path) {
  auto conf = IniAccess(ini_path);
  const char* section = "GRAVITY_GRADIENT";

  GravityGradient ggdist;
  ggdist.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  ggdist.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return ggdist;
}

GravityGradient InitGravityGradient(std::string ini_path, const double mu_m3_s2) {
  auto conf = IniAccess(ini_path);
  const char* section = "GRAVITY_GRADIENT";

  GravityGradient ggdist(mu_m3_s2);
  ggdist.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  ggdist.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return ggdist;
}

MagDisturbance InitMagDisturbance(std::string ini_path, const RMMParams& rmm_params) {
  auto conf = IniAccess(ini_path);
  const char* section = "MAGNETIC_DISTURBANCE";

  MagDisturbance mag_dist(rmm_params);
  mag_dist.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  mag_dist.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return mag_dist;
}

GeoPotential InitGeoPotential(std::string ini_path) {
  auto conf = IniAccess(ini_path);
  const char* section = "GEOPOTENTIAL";

  int degree = conf.ReadInt(section, "degree");
  std::string file_path = conf.ReadString(section, "coefficients_file_path");
  GeoPotential geop(degree, file_path);
  geop.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  geop.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return geop;
}

ThirdBodyGravity InitThirdBodyGravity(std::string ini_path, std::string ini_path_celes) {
  // Generate a list of bodies to be calculated in "CelesInfo"
  auto conf_celes = IniAccess(ini_path_celes);
  const char* section_celes = "CELESTIAL_INFORMATION";
  const int num_of_selected_body = conf_celes.ReadInt(section_celes, "number_of_selected_body");
  std::string center_object = conf_celes.ReadString(section_celes, "center_object");
  std::set<std::string> selected_body_list;

  for (int i = 0; i < num_of_selected_body; i++) {
    std::string selected_body_id = "selected_body_name(" + std::to_string(i) + ")";
    selected_body_list.insert(conf_celes.ReadString(section_celes, selected_body_id.c_str()));
  }

  // Generate a list of bodies to be calculated in "ThirdBodyGravity" from the list of bodies of "CelesInfo"
  auto conf = IniAccess(ini_path);
  const char* section = "THIRD_BODY_GRAVITY";

  const int num_of_third_body = conf.ReadInt(section, "number_of_third_body");
  std::set<std::string> third_body_list;

  // Generate the list of the third object if "calculation=ENABLE"
  if (conf.ReadEnable(section, CALC_LABEL)) {
    for (int i = 0; i < num_of_third_body; i++) {
      std::string third_body_id = "third_body_name(" + std::to_string(i) + ")";
      std::string third_body_name = conf.ReadString(section, third_body_id.c_str());
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

  ThirdBodyGravity thirdbodyg(third_body_list);
  thirdbodyg.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  thirdbodyg.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return thirdbodyg;
}
