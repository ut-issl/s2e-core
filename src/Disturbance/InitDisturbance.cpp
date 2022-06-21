#include "InitDisturbance.hpp"

#include <Interface/InitInput/IniAccess.h>

#define CALC_LABEL "calculation"
#define LOG_LABEL "logging"
#define MIN_VAL 1e-9

AirDrag InitAirDrag(std::string ini_path, const std::vector<Surface>& surfaces, const Vector<3> cg_b) {
  auto conf = IniAccess(ini_path);
  const char* section = "AIRDRAG";

  double t_w = conf.ReadDouble(section, "Temp_wall") + 273.0;
  double t_m = conf.ReadDouble(section, "Temp_molecular") + 273.0;
  double molecular = conf.ReadDouble(section, "Molecular");

  bool calcen = conf.ReadEnable(section, CALC_LABEL);
  bool logen = conf.ReadEnable(section, LOG_LABEL);

  AirDrag airdrag(surfaces, cg_b, t_w, t_m, molecular);
  airdrag.IsCalcEnabled = calcen;
  airdrag.IsLogEnabled = logen;

  return airdrag;
}

SolarRadiation InitSRDist(std::string ini_path, const std::vector<Surface>& surfaces, const Vector<3> cg_b) {
  auto conf = IniAccess(ini_path);
  const char* section = "SRDIST";

  bool calcen = conf.ReadEnable(section, CALC_LABEL);
  bool logen = conf.ReadEnable(section, LOG_LABEL);

  SolarRadiation srdist(surfaces, cg_b);
  srdist.IsCalcEnabled = calcen;
  srdist.IsLogEnabled = logen;

  return srdist;
}

GravityGradient InitGGDist(std::string ini_path) {
  auto conf = IniAccess(ini_path);
  const char* section = "GRAVITY_GRADIENT";

  GravityGradient ggdist;
  ggdist.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  ggdist.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return ggdist;
}

MagDisturbance InitMagDisturbance(std::string ini_path, RMMParams rmm_params) {
  auto conf = IniAccess(ini_path);
  const char* section = "MAG_DISTURBANCE";

  MagDisturbance mag_dist(rmm_params.GetRMMConst_b(), rmm_params.GetRMMRWDev(), rmm_params.GetRMMRWLimit(), rmm_params.GetRMMWNVar());
  mag_dist.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  mag_dist.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return mag_dist;
}

GeoPotential InitGeoPotential(std::string ini_path) {
  auto conf = IniAccess(ini_path);
  const char* section = "GEOPOTENTIAL";

  int degree = conf.ReadInt(section, "degree");
  std::string file_path = conf.ReadString(section, "file_path");
  GeoPotential geop(degree, file_path);
  geop.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  geop.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return geop;
}

ThirdBodyGravity InitThirdBodyGravity(std::string ini_path, std::string ini_path_celes) {
  // Generate a list of bodies to be calculated in "CelesInfo"
  auto conf_celes = IniAccess(ini_path_celes);
  const char* section_celes = "PLANET_SELECTION";
  const int num_of_selected_body = conf_celes.ReadInt(section_celes, "num_of_selected_body");
  std::string center_object = conf_celes.ReadString(section_celes, "center_object");
  std::set<std::string> selected_body_list;

  for (int i = 0; i < num_of_selected_body; i++) {
    std::string selected_body_id = "selected_body(" + std::to_string(i) + ")";
    selected_body_list.insert(conf_celes.ReadString(section_celes, selected_body_id.c_str()));
  }

  // Generate a list of bodies to be calculated in "ThirdBodyGravity" from the
  // list of bodies of "CelesInfo"
  auto conf = IniAccess(ini_path);
  const char* section = "THIRD_BODY_GRAVITY";

  const int num_of_third_body = conf.ReadInt(section, "num_of_third_body");
  std::set<std::string> third_body_list;

  if (conf.ReadEnable(section,
                      CALC_LABEL))  // Generate the list of the third object if
                                    // and only if "calculation=ENABLE"
  {
    for (int i = 0; i < num_of_third_body; i++) {
      std::string third_body_id = "third_body(" + std::to_string(i) + ")";
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
