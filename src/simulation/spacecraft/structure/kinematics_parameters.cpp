/**
 * @file kinematics_parameters.cpp
 * @brief Definition of Kinematics information
 */

#include "kinematics_parameters.hpp"

namespace s2e::simulation {

KinematicsParameters::KinematicsParameters(s2e::math::Vector<3> center_of_gravity_b_m, double mass_kg, s2e::math::Matrix<3, 3> inertia_tensor_b_kgm2)
    : center_of_gravity_b_m_(center_of_gravity_b_m), mass_kg_(mass_kg), inertia_tensor_b_kgm2_(inertia_tensor_b_kgm2) {}

} // namespace s2e::simulation
