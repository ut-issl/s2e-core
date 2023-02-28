/*
 * @file initialize_simple_thruster.cpp
 * @brief Initialize function os SimpleThruster
 */
#include "initialize_simple_thruster.hpp"

#include <library/math/constants.hpp>

#include "library/initialize/initialize_file_access.hpp"

SimpleThruster InitSimpleThruster(ClockGenerator* clock_generator, int thruster_id, const std::string file_name, const Structure* structure,
                                  const Dynamics* dynamics) {
  IniAccess thruster_conf(file_name);
  std::string sectionstr = "THRUSTER_" + std::to_string(thruster_id);
  auto* Section = sectionstr.c_str();

  int prescaler = thruster_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<3> thruster_pos;
  thruster_conf.ReadVector(Section, "thruster_position_b_m", thruster_pos);

  Vector<3> thruster_dir;
  thruster_conf.ReadVector(Section, "thruster_direction_b", thruster_dir);

  double max_mag = thruster_conf.ReadDouble(Section, "thrust_magnitude_N");

  double mag_err;
  mag_err = thruster_conf.ReadDouble(Section, "thrust_error_standard_deviation_N");

  double deg_err;
  deg_err = thruster_conf.ReadDouble(Section, "direction_error_standard_deviation_deg") * libra::pi / 180.0;

  SimpleThruster thruster(prescaler, clock_generator, thruster_id, thruster_pos, thruster_dir, max_mag, mag_err, deg_err, structure, dynamics);
  return thruster;
}

SimpleThruster InitSimpleThruster(ClockGenerator* clock_generator, PowerPort* power_port, int thruster_id, const std::string file_name,
                                  const Structure* structure, const Dynamics* dynamics) {
  IniAccess thruster_conf(file_name);
  std::string sectionstr = "THRUSTER_" + std::to_string(thruster_id);
  auto* Section = sectionstr.c_str();

  int prescaler = thruster_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<3> thruster_pos;
  thruster_conf.ReadVector(Section, "thruster_position_b_m", thruster_pos);

  Vector<3> thruster_dir;
  thruster_conf.ReadVector(Section, "thruster_direction_b", thruster_dir);

  double max_mag = thruster_conf.ReadDouble(Section, "thrust_magnitude_N");

  double mag_err;
  mag_err = thruster_conf.ReadDouble(Section, "thrust_error_standard_deviation_N");

  double deg_err;
  deg_err = thruster_conf.ReadDouble(Section, "direction_error_standard_deviation_deg") * libra::pi / 180.0;

  power_port->InitializeWithInitializeFile(file_name);

  SimpleThruster thruster(prescaler, clock_generator, power_port, thruster_id, thruster_pos, thruster_dir, max_mag, mag_err, deg_err, structure,
                          dynamics);
  return thruster;
}
