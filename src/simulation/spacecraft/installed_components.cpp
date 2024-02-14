/**
 * @file installed_components.cpp
 * @brief Definition of InstalledComponents class
 */

#include "installed_components.hpp"

#include <utilities/macros.hpp>

libra::Vector<3> InstalledComponents::GenerateForce_b_N() {
  libra::Vector<3> force_b_N_(0.0);
  return force_b_N_;
}

libra::Vector<3> InstalledComponents::GenerateTorque_b_Nm() {
  libra::Vector<3> torque_b_Nm_(0.0);
  return torque_b_Nm_;
}

void InstalledComponents::LogSetup(Logger& logger) { UNUSED(logger); }
