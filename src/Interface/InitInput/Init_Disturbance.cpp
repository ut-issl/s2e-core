#include "Initialize.h"
#include "../../Disturbance/AirDrag.h"
#include "../../Disturbance/SolarRadiation.h"
#include "../../Disturbance/GGDist.h"
#include "../../Disturbance/MagDisturbance.h"
#include "../../Disturbance/GeoPotential.h"
#include "../../Disturbance/ThirdBodyGravity.h"

#define MIN_VAL 1e-9

vector<Surface> InitSurfaces(string ini_path)
{
  auto conf = IniAccess(ini_path);
  char* section = "SURFACES";

  const int num_surface = conf.ReadInt(section, "num_of_surfaces");
  vector<Surface> surfaces;

  for (int i=0;i<num_surface;i++)
  {
    string idx = std::to_string(i);
    idx = "_" + idx;
    string keyword;

    keyword = "area"+idx;
    double area = conf.ReadDouble(section, keyword.c_str());
    if(area < -MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    }

    keyword = "reflectivity"+idx;
    double ref = conf.ReadDouble(section, keyword.c_str());
    if(ref < -MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    }
    else if(ref > 1.0+MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": larger than 1.0\n";
      break;
    }

    keyword = "specularity"+idx;
    double spe = conf.ReadDouble(section, keyword.c_str());
    if(spe < -MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    }
    else if(spe > 1.0+MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": larger than 0.0\n";
      break;
    }

    keyword = "air_specularity"+idx;   
    double air_spe = conf.ReadDouble(section, keyword.c_str());
    if(air_spe < -MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    }
    else if(air_spe > 1.0+MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": larger than 0.0\n";
      break;
    }

    Vector<3> position, normal;
    keyword = "position"+idx;   
    conf.ReadVector(section, keyword.c_str(), position);

    keyword = "normal"+idx;   
    conf.ReadVector(section, keyword.c_str(), normal);
    if(norm(normal) > 1.0+MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": norm is larger than 1.0\n";
      break;
    }

    //Add a surface
    surfaces.push_back(Surface(position, normal, area, ref, spe, air_spe));
  }
  return surfaces;
}

AirDrag InitAirDrag(string ini_path, const vector<Surface>& surfaces)
{
  auto conf = IniAccess(ini_path);
  char* section = "AIRDRAG";

  double t_w = conf.ReadDouble(section, "Temp_wall") + 273.0;
  double t_m = conf.ReadDouble(section, "Temp_molecular") + 273.0;
  double molecular = conf.ReadDouble(section, "Molecular");

  bool calcen = conf.ReadEnable(section, CALC_LABEL);
  bool logen = conf.ReadEnable(section, LOG_LABEL);

  section = "SURFACEFORCE";
  Vector<3> center(0);
  conf.ReadVector(section, "center", center);
  AirDrag airdrag(surfaces, center, t_w, t_m, molecular);
  airdrag.IsCalcEnabled = calcen;
  airdrag.IsLogEnabled = logen;

  return airdrag;
}

SolarRadiation InitSRDist(string ini_path, const vector<Surface>& surfaces)
{
  auto conf = IniAccess(ini_path);
  char* section = "SRDIST";

  bool calcen = conf.ReadEnable(section, CALC_LABEL);
  bool logen = conf.ReadEnable(section, LOG_LABEL);

  section = "SURFACEFORCE";

  Vector<3> center(0);
  conf.ReadVector(section, "center", center);
  SolarRadiation srdist(surfaces, center);
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