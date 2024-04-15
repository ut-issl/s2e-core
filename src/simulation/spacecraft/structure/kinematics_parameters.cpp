/**
 * @file kinematics_parameters.cpp
 * @brief Definition of Kinematics information
 */

#include "kinematics_parameters.hpp"

KinematicsParameters::KinematicsParameters(libra::Vector<3> center_of_gravity_b_m, double mass_kg, libra::Matrix<3, 3> inertia_tensor_b_kgm2)
    : center_of_gravity_b_m_(center_of_gravity_b_m), mass_kg_(mass_kg), inertia_tensor_b_kgm2_(inertia_tensor_b_kgm2) {}

KinematicsParameters::KinematicsParameters(libra::Vector<3> center_of_gravity_b_m, double mass_kg, libra::Matrix<3, 3> inertia_tensor_b_kgm2,
                                           libra::Matrix<3, 3> inertia_tensor_flexible_b_kgm2, double zeta_flexible, double omega_flexible_rad_s)
    : center_of_gravity_b_m_(center_of_gravity_b_m),
      mass_kg_(mass_kg),
      inertia_tensor_b_kgm2_(inertia_tensor_b_kgm2),
      inertia_tensor_flexible_b_kgm2_(inertia_tensor_flexible_b_kgm2),
      zeta_flexible_(zeta_flexible),
      omega_flexible_rad_s_(omega_flexible_rad_s) {}