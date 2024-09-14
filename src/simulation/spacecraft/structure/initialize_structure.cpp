/**
 * @file initialize_structure.cpp
 * @brief Initialize functions for spacecraft structure
 */

#include "initialize_structure.hpp"

#include <math_physics/math/vector.hpp>
#include <setting_file_reader/initialize_file_access.hpp>

#define MIN_VAL 1e-6
KinematicsParameters InitKinematicsParameters(std::string file_name) {
  auto conf = IniAccess(file_name);
  const char* section = "KINEMATIC_PARAMETERS";

  s2e::math::Vector<3> center_of_gravity_b_m;
  conf.ReadVector(section, "center_of_gravity_b_m", center_of_gravity_b_m);
  double mass_kg = conf.ReadDouble(section, "mass_kg");
  s2e::math::Vector<9> inertia_vec;
  s2e::math::Matrix<3, 3> inertia_tensor_b_kgm2;
  conf.ReadVector(section, "inertia_tensor_kgm2", inertia_vec);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      inertia_tensor_b_kgm2[i][j] = inertia_vec[i * 3 + j];
    }
  }

  KinematicsParameters kinematics_params(center_of_gravity_b_m, mass_kg, inertia_tensor_b_kgm2);
  return kinematics_params;
}

std::vector<Surface> InitSurfaces(std::string file_name) {
  using std::cout;

  auto conf = IniAccess(file_name);
  const char* section = "SURFACES";

  const int num_surface = conf.ReadInt(section, "number_of_surfaces");
  std::vector<Surface> surfaces;

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
    if (normal.CalcNorm() > 1.0 + MIN_VAL)  // Fixme: magic word
    {
      cout << "Surface Warning! " << keyword << ": norm is larger than 1.0.";
      cout << "The vector is normalized.\n";
      normal = normal.CalcNormalizedVector();
    }

    // Add a surface
    surfaces.push_back(Surface(position, normal, area, ref, spe, air_spe));
  }
  return surfaces;
}

ResidualMagneticMoment InitResidualMagneticMoment(std::string file_name) {
  auto conf = IniAccess(file_name);
  const char* section = "RESIDUAL_MAGNETIC_MOMENT";

  s2e::math::Vector<3> rmm_const_b;
  conf.ReadVector(section, "rmm_constant_b_Am2", rmm_const_b);
  double rmm_rwdev = conf.ReadDouble(section, "rmm_random_walk_speed_Am2");
  double random_walk_limit_Am2 = conf.ReadDouble(section, "rmm_random_walk_limit_Am2");
  double random_noise_standard_deviation_Am2 = conf.ReadDouble(section, "rmm_white_noise_standard_deviation_Am2");

  ResidualMagneticMoment rmm_params(rmm_const_b, rmm_rwdev, random_walk_limit_Am2, random_noise_standard_deviation_Am2);
  return rmm_params;
}
