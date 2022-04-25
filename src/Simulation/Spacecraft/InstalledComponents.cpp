#include "InstalledComponents.hpp"

#include <Library/utils/Macros.hpp>

libra::Vector<3> InstalledComponents::GenerateForce_N_b() {
  libra::Vector<3> force_N_b_(0.0);
  return force_N_b_;
};

libra::Vector<3> InstalledComponents::GenerateTorque_Nm_b() {
  libra::Vector<3> torque_Nm_b_(0.0);
  return torque_Nm_b_;
};

void InstalledComponents::LogSetup(Logger& logger) { UNUSED(logger); }
