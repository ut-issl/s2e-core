/*
 * @file InitializeTorqueGenerator.cpp
 * @brief Initialize function for TorqueGenerator
 */
#include <Interface/InitInput/IniAccess.h>

#include "InitializeTorqueGenerator.hpp"

TorqueGenerator InitializeTorqueGenerator(ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("ComponentBase", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // TorqueGenerator
  char section[30] = "TorqueGenerator";
  double torque_magnitude_standard_deviation_N = ini_file.ReadDouble(section, "torque_magnitude_standard_deviation_N");
  double torque_direction_standard_deviation_deg = ini_file.ReadDouble(section, "torque_direction_standard_deviation_deg");
  double torque_direction_standard_deviation_rad = libra::deg_to_rad * torque_direction_standard_deviation_deg;
  TorqueGenerator torque_generator(prescaler, clock_gen, torque_magnitude_standard_deviation_N, torque_direction_standard_deviation_rad, dynamics);

  return torque_generator;
}