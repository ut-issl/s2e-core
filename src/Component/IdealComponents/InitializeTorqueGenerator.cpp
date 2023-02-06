/*
 * @file InitializeTorqueGenerator.cpp
 * @brief Initialize function for TorqueGenerator
 */
#include "InitializeTorqueGenerator.hpp"

#include <Interface/InitInput/IniAccess.h>

TorqueGenerator InitializeTorqueGenerator(ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("COMPONENT_BASE", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // TorqueGenerator
  char section[30] = "TORQUE_GENERATOR";
  double torque_magnitude_standard_deviation_Nm = ini_file.ReadDouble(section, "torque_magnitude_standard_deviation_Nm");
  double torque_direction_standard_deviation_deg = ini_file.ReadDouble(section, "torque_direction_standard_deviation_deg");
  double torque_direction_standard_deviation_rad = libra::deg_to_rad * torque_direction_standard_deviation_deg;
  TorqueGenerator torque_generator(prescaler, clock_gen, torque_magnitude_standard_deviation_Nm, torque_direction_standard_deviation_rad, dynamics);

  return torque_generator;
}
