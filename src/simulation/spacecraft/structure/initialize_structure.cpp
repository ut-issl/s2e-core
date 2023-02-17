/**
 * @file initialize_structure.cpp
 * @brief Initialize functions for spacecraft structure
 */

#include "initialize_structure.hpp"

#include <library/initialize/initialize_file_access.hpp>
#include <library/math/vector.hpp>

#define MIN_VAL 1e-6
KinematicsParams InitKinematicsParams(std::string ini_path) {
  auto conf = IniAccess(ini_path);
  const char* section = "KINEMATIC_PARAMETERS";

  Vector<3> cg_b;
  conf.ReadVector(section, "center_of_gravity_b_m", cg_b);
  double mass = conf.ReadDouble(section, "mass_kg");
  Vector<9> inertia_vec;
  Matrix<3, 3> inertia_tensor;
  conf.ReadVector(section, "inertia_tensor_kgm2", inertia_vec);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      inertia_tensor[i][j] = inertia_vec[i * 3 + j];
    }
  }

  KinematicsParams kinematics_params(cg_b, mass, inertia_tensor);
  return kinematics_params;
}

vector<Surface> InitSurfaces(std::string ini_path) {
  using std::cout;

  auto conf = IniAccess(ini_path);
  const char* section = "SURFACES";

  const int num_surface = conf.ReadInt(section, "number_of_surfaces");
  vector<Surface> surfaces;

  for (int i = 0; i < num_surface; i++) {
    std::string idx = std::to_string(i);
    idx = "_" + idx;
    std::string keyword;

    keyword = "area" + idx + "_m2";
    double area = conf.ReadDouble(section, keyword.c_str());
    if (area < -MIN_VAL)  // Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    }

    keyword = "reflectivity" + idx;
    double ref = conf.ReadDouble(section, keyword.c_str());
    if (ref < -MIN_VAL)  // Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    } else if (ref > 1.0 + MIN_VAL)  // Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": larger than 1.0\n";
      break;
    }

    keyword = "specularity" + idx;
    double spe = conf.ReadDouble(section, keyword.c_str());
    if (spe < -MIN_VAL)  // Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    } else if (spe > 1.0 + MIN_VAL)  // Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": larger than 0.0\n";
      break;
    }

    keyword = "air_specularity" + idx;
    double air_spe = conf.ReadDouble(section, keyword.c_str());
    if (air_spe < -MIN_VAL)  // Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": smaller than 0.0\n";
      break;
    } else if (air_spe > 1.0 + MIN_VAL)  // Fixme: magic word
    {
      cout << "Surface Error! " << keyword << ": larger than 0.0\n";
      break;
    }

    Vector<3> position, normal;
    keyword = "position" + idx + "_b_m";
    conf.ReadVector(section, keyword.c_str(), position);

    keyword = "normal_vector" + idx + "_b";
    conf.ReadVector(section, keyword.c_str(), normal);
    if (norm(normal) > 1.0 + MIN_VAL)  // Fixme: magic word
    {
      cout << "Surface Warning! " << keyword << ": norm is larger than 1.0.";
      cout << "The vector is normalized.\n";
      normal = normalize(normal);
    }

    // Add a surface
    surfaces.push_back(Surface(position, normal, area, ref, spe, air_spe));
  }
  return surfaces;
}

RMMParams InitRMMParams(std::string ini_path) {
  auto conf = IniAccess(ini_path);
  const char* section = "RESIDUAL_MAGNETIC_MOMENT";

  Vector<3> rmm_const_b;
  conf.ReadVector(section, "rmm_constant_b_Am2", rmm_const_b);
  double rmm_rwdev = conf.ReadDouble(section, "rmm_random_walk_speed_Am2");
  double rmm_rwlimit = conf.ReadDouble(section, "rmm_random_walk_limit_Am2");
  double rmm_wnvar = conf.ReadDouble(section, "rmm_white_noise_standard_deviation_Am2");

  RMMParams rmm_params(rmm_const_b, rmm_rwdev, rmm_rwlimit, rmm_wnvar);
  return rmm_params;
}
