/**
 * @file kinematics_parameters.cpp
 * @brief Definition of Kinematics information
 */

#include "kinematics_parameters.hpp"

KinematicsParams::KinematicsParams(libra::Vector<3> cg_b, double mass, libra::Matrix<3, 3> inertia_tensor)
    : center_of_gravity_b_m_(cg_b), mass_kg_(mass), inertia_tensor_b_kgm2_(inertia_tensor) {}