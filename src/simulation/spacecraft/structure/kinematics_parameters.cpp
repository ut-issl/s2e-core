/**
 * @file kinematics_parameters.cpp
 * @brief Definition of Kinematics information
 */

#include "kinematics_parameters.hpp"

KinematicsParameters::KinematicsParameters(libra::Vector<3> center_of_gravity_b_m, double mass_kg, libra::Matrix<3, 3> inertia_tensor_b_kgm2)
    : center_of_gravity_b_m_(center_of_gravity_b_m), mass_kg_(mass_kg), inertia_tensor_b_kgm2_(inertia_tensor_b_kgm2) {}