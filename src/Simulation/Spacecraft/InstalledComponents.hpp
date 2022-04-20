#pragma once

#include <Interface/LogOutput/Logger.h>

#include <Library/math/Vector.hpp>

class InstalledComponents {
 public:
  virtual libra::Vector<3> GenerateForce_N_b();
  virtual libra::Vector<3> GenerateTorque_Nm_b();
  virtual void LogSetup(Logger& logger);
};
