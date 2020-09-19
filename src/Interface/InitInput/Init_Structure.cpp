#include "Initialize.h"
#include"../../Simulation/Spacecraft/Structure/KinematicsParams.h"
#include"../../Simulation/Spacecraft/Structure/Surface.h"
#include"../../Simulation/Spacecraft/Structure/RMMParams.h"

#define MIN_VAL 1e-9
KinematicsParams InitKinematicsParams(string ini_path)
{
  auto conf = IniAccess(ini_path);
  char* section = "STRUCTURE";

  Vector<3> cg_b;
  conf.ReadVector(section, "cg_b", cg_b);
  double mass = conf.ReadDouble(section, "mass");
  Vector<9> inertia_vec;
  Matrix<3, 3> inertia_tensor;
  conf.ReadVector(section, "Iner", inertia_vec);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      inertia_tensor[i][j] = inertia_vec[i * 3 + j];
    }
  }

  KinematicsParams kinematics_params(cg_b, mass, inertia_tensor);
  return kinematics_params;
}

vector<Surface> InitSurfaces(string ini_path)
{
  auto conf = IniAccess(ini_path);
  char* section = "SURFACES";

  const int num_surface = conf.ReadInt(section, "num_of_surfaces");
  vector<Surface> surfaces;

  for (int i = 0; i < num_surface; i++)
  {
    string idx = std::to_string(i);
    idx = "_" + idx;
    string keyword;

    keyword = "area" + idx;
    double area = conf.ReadDouble(section, keyword.c_str());
    if (area < -MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    }

    keyword = "reflectivity" + idx;
    double ref = conf.ReadDouble(section, keyword.c_str());
    if (ref < -MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    }
    else if (ref > 1.0 + MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": larger than 1.0\n";
      break;
    }

    keyword = "specularity" + idx;
    double spe = conf.ReadDouble(section, keyword.c_str());
    if (spe < -MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    }
    else if (spe > 1.0 + MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": larger than 0.0\n";
      break;
    }

    keyword = "air_specularity" + idx;
    double air_spe = conf.ReadDouble(section, keyword.c_str());
    if (air_spe < -MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    }
    else if (air_spe > 1.0 + MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": larger than 0.0\n";
      break;
    }

    Vector<3> position, normal;
    keyword = "position" + idx;
    conf.ReadVector(section, keyword.c_str(), position);

    keyword = "normal" + idx;
    conf.ReadVector(section, keyword.c_str(), normal);
    if (norm(normal) > 1.0 + MIN_VAL) //Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": norm is larger than 1.0\n";
      break;
    }

    //Add a surface
    surfaces.push_back(Surface(position, normal, area, ref, spe, air_spe));
  }
  return surfaces;
}


RMMParams InitRMMParams(string ini_path)
{
  auto conf = IniAccess(ini_path);
  char* section = "RMM";

  Vector<3> rmm_const_b;
  conf.ReadVector(section, "rmm_const_b", rmm_const_b);
  double rmm_rwdev = conf.ReadDouble(section, "rmm_rwdev");
  double rmm_rwlimit = conf.ReadDouble(section, "rmm_rwlimit");
  double rmm_wnvar = conf.ReadDouble(section, "rmm_wnvar");

  RMMParams rmm_params(rmm_const_b, rmm_rwdev, rmm_rwlimit, rmm_wnvar);
  return rmm_params;
}