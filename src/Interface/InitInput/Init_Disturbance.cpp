#include "Initialize.h"
#include "../../Disturbance/AirDrag.h"
#include "../../Disturbance/SolarRadiation.h"
#include "../../Disturbance/GGDist.h"
#include "../../Disturbance/MagDisturbance.h"
#include "../../Disturbance/GeoPotential.h"
#include "../../Disturbance/ThirdBodyGravity.h"

AirDrag InitAirDrag(string ini_path)
{
  auto conf = IniAccess(ini_path);
  char* section = "AIRDRAG";

  //air drag
  Vector<6> specularity(0);
  //surface force
  Vector<3> px_arm(0), mx_arm(0), py_arm(0), my_arm(0), pz_arm(0), mz_arm(0);
  Vector<6> area(0);
  Vector<3> px_normal(0), mx_normal(0), py_normal(0), my_normal(0), pz_normal(0), mz_normal(0);
  Vector<3> center(0);

  double t_w = conf.ReadDouble(section, "Temp_wall") + 273.0;
  double t_m = conf.ReadDouble(section, "Temp_molecular") + 273.0;
  double molecular = conf.ReadDouble(section, "Molecular");
  conf.ReadVector(section, "specularity", specularity);
  bool calcen = conf.ReadEnable(section, CALC_LABEL);
  bool logen = conf.ReadEnable(section, LOG_LABEL);

  section = "SURFACEFORCE";

  conf.ReadVector(section, "px_arm", px_arm);
  conf.ReadVector(section, "mx_arm", mx_arm);
  conf.ReadVector(section, "py_arm", py_arm);
  conf.ReadVector(section, "my_arm", my_arm);
  conf.ReadVector(section, "pz_arm", pz_arm);
  conf.ReadVector(section, "mz_arm", mz_arm);

  conf.ReadVector(section, "area", area);

  conf.ReadVector(section, "px_normal", px_normal);
  conf.ReadVector(section, "mx_normal", mx_normal);
  conf.ReadVector(section, "py_normal", py_normal);
  conf.ReadVector(section, "my_normal", my_normal);
  conf.ReadVector(section, "pz_normal", pz_normal);
  conf.ReadVector(section, "mz_normal", mz_normal);

  conf.ReadVector(section, "center", center);

  AirDrag airdrag(px_arm, mx_arm, py_arm, my_arm, pz_arm, mz_arm, area,
    px_normal, mx_normal, py_normal, my_normal, pz_normal, mz_normal, center, specularity, t_w, t_m, molecular);
  airdrag.IsCalcEnabled = calcen;
  airdrag.IsLogEnabled = logen;

  return airdrag;
}

SolarRadiation InitSRDist(string ini_path)
{
  auto conf = IniAccess(ini_path);
  char* section = "SRDIST";

  //SRP
  Vector<6> reflectivity(0), specularity(0);
  //surface force
  Vector<3> px_arm(0), mx_arm(0), py_arm(0), my_arm(0), pz_arm(0), mz_arm(0);
  Vector<6> area(0);
  Vector<3> px_normal(0), mx_normal(0), py_normal(0), my_normal(0), pz_normal(0), mz_normal(0);
  Vector<3> center(0);

  conf.ReadVector(section, "reflectivity", reflectivity);
  conf.ReadVector(section, "specularity", specularity);
  bool calcen = conf.ReadEnable(section, CALC_LABEL);
  bool logen = conf.ReadEnable(section, LOG_LABEL);

  section = "SURFACEFORCE";

  conf.ReadVector(section, "px_arm", px_arm);
  conf.ReadVector(section, "mx_arm", mx_arm);
  conf.ReadVector(section, "py_arm", py_arm);
  conf.ReadVector(section, "my_arm", my_arm);
  conf.ReadVector(section, "pz_arm", pz_arm);
  conf.ReadVector(section, "mz_arm", mz_arm);

  conf.ReadVector(section, "area", area);

  conf.ReadVector(section, "px_normal", px_normal);
  conf.ReadVector(section, "mx_normal", mx_normal);
  conf.ReadVector(section, "py_normal", py_normal);
  conf.ReadVector(section, "my_normal", my_normal);
  conf.ReadVector(section, "pz_normal", pz_normal);
  conf.ReadVector(section, "mz_normal", mz_normal);

  conf.ReadVector(section, "center", center);
  SolarRadiation srdist(px_arm, mx_arm, py_arm, my_arm, pz_arm, mz_arm, area,
    px_normal, mx_normal, py_normal, my_normal, pz_normal, mz_normal, center, reflectivity, specularity);
  srdist.IsCalcEnabled = calcen;
  srdist.IsLogEnabled = logen;

  return srdist;
}

GGDist InitGGDist(string ini_path)
{
  auto conf = IniAccess(ini_path);
  const char* section = "GRAVITY_GRADIENT";

  GGDist ggdist;
  ggdist.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  ggdist.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return ggdist;
}

MagDisturbance InitMagDisturbance(string ini_path)
{
  auto conf = IniAccess(ini_path);
  char* section = "MAG_DISTURBANCE";

  Vector<3> rmm_const_b(0);
  double rmm_rwdev = 0.0, rmm_rwlimit = 0.0, rmm_wnvar = 0.0;

  conf.ReadVector(section, "rmm_const_b", rmm_const_b);

  rmm_rwdev = conf.ReadDouble(section, "rmm_rwdev");
  rmm_rwlimit = conf.ReadDouble(section, "rmm_rwlimit");
  rmm_wnvar = conf.ReadDouble(section, "rmm_wnvar");

  MagDisturbance mag_dist(rmm_const_b, rmm_rwdev, rmm_rwlimit, rmm_wnvar);
  mag_dist.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  mag_dist.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return mag_dist;
}

GeoPotential InitGeoPotential(string ini_path)
{
  auto conf = IniAccess(ini_path);
  const char* section = "GEOPOTENTIAL";

  int degree = conf.ReadInt(section, "degree");
  string file_path = conf.ReadString(section, "file_path");
  GeoPotential geop(degree, file_path);
  geop.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  geop.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return geop;
}

ThirdBodyGravity InitThirdBodyGravity(string ini_path, string ini_path_celes)
{
  //Generate a list of bodies to be calculated in "CelesInfo"
  auto conf_celes = IniAccess(ini_path_celes);
  const char* section_celes = "PLANET_SELECTION";
  const int num_of_selected_body = conf_celes.ReadInt(section_celes, "num_of_selected_body");
  string center_object = conf_celes.ReadString(section_celes, "center_object");
  std::set<string> selected_body_list;

  for (int i = 0; i < num_of_selected_body; i++) {
    string selected_body_id = "selected_body(" + to_string(i) + ")";
    selected_body_list.insert(conf_celes.ReadString(section_celes, selected_body_id.c_str()));
  }

  //Generate a list of bodies to be calculated in "ThirdBodyGravity" from the list of bodies of "CelesInfo"
  auto conf = IniAccess(ini_path);
  const char* section = "THIRD_BODY_GRAVITY";

  const int num_of_third_body = conf.ReadInt(section, "num_of_third_body");
  std::set<string> third_body_list;

  if (conf.ReadEnable(section, CALC_LABEL)) //Generate the list of the third object if and only if "calculation=ENABLE"
  {
    for (int i = 0; i < num_of_third_body; i++) {
      string third_body_id = "third_body(" + to_string(i) + ")";
      string third_body_name = conf.ReadString(section, third_body_id.c_str());
      //If the object specified by `third_body` in "SampleDisturbance.ini" is the center object of the orbital propagation, the system prints an error message.
      assert(third_body_name != center_object);
      //If the target specified by `third_body` in "SampleDisturbance.ini" is not in the list of bodies to be calculated by "CelesInfo", the system prints an error message.
      assert(selected_body_list.find(third_body_name) != selected_body_list.end());
      third_body_list.insert(third_body_name);
    }
  }

  ThirdBodyGravity thirdbodyg(third_body_list);
  thirdbodyg.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  thirdbodyg.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return thirdbodyg;
}