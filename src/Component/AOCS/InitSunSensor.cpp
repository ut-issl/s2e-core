/**
 * @file InitSunSensor.cpp
 * @brief Initialize functions for sun sensor
 */
#include "InitSunSensor.hpp"

#include <string.h>

#include <Library/math/Constant.hpp>

#include "Interface/InitInput/IniAccess.h"

SunSensor InitSunSensor(ClockGenerator* clock_gen, int ss_id, std::string file_name, const SRPEnvironment* srp,
                        const LocalCelestialInformation* local_celes_info) {
  IniAccess ss_conf(file_name);
  const char* sensor_name = "SUNSENSOR";
  const std::string section_tmp = sensor_name + std::to_string(static_cast<long long>(ss_id));
  const char* Section = section_tmp.c_str();

  int prescaler = ss_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Quaternion q_b2c;
  ss_conf.ReadQuaternion(Section, "q_b2c", q_b2c);

  double detectable_angle_deg = 0.0, detectable_angle_rad = 0.0;
  detectable_angle_deg = ss_conf.ReadDouble(Section, "detectable_angle_deg");
  detectable_angle_rad = libra::pi / 180.0 * detectable_angle_deg;

  double nr_stddev = 0.0;
  nr_stddev = ss_conf.ReadDouble(Section, "nr_stddev");
  nr_stddev *= libra::pi / 180.0;

  double nr_bias_stddev = 0.0;
  nr_bias_stddev = ss_conf.ReadDouble(Section, "nr_bias_stddev");
  nr_bias_stddev *= libra::pi / 180.0;

  double intensity_lower_threshold_percent;
  intensity_lower_threshold_percent = ss_conf.ReadDouble(Section, "intensity_lower_threshold_percent");

  SunSensor ss(prescaler, clock_gen, ss_id, q_b2c, detectable_angle_rad, nr_stddev, nr_bias_stddev, intensity_lower_threshold_percent, srp,
               local_celes_info);
  return ss;
}

SunSensor InitSunSensor(ClockGenerator* clock_gen, PowerPort* power_port, int ss_id, std::string file_name, const SRPEnvironment* srp,
                        const LocalCelestialInformation* local_celes_info) {
  IniAccess ss_conf(file_name);
  const char* sensor_name = "SUNSENSOR";
  const std::string section_tmp = sensor_name + std::to_string(static_cast<long long>(ss_id));
  const char* Section = section_tmp.c_str();

  int prescaler = ss_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Quaternion q_b2c;
  ss_conf.ReadQuaternion(Section, "q_b2c", q_b2c);

  double detectable_angle_deg = 0.0, detectable_angle_rad = 0.0;
  detectable_angle_deg = ss_conf.ReadDouble(Section, "detectable_angle_deg");
  detectable_angle_rad = libra::pi / 180.0 * detectable_angle_deg;

  double nr_stddev = 0.0;
  nr_stddev = ss_conf.ReadDouble(Section, "nr_stddev");
  nr_stddev *= libra::pi / 180.0;

  double nr_bias_stddev = 0.0;
  nr_bias_stddev = ss_conf.ReadDouble(Section, "nr_bias_stddev");
  nr_bias_stddev *= libra::pi / 180.0;

  double intensity_lower_threshold_percent;
  intensity_lower_threshold_percent = ss_conf.ReadDouble(Section, "intensity_lower_threshold_percent");

  SunSensor ss(prescaler, clock_gen, power_port, ss_id, q_b2c, detectable_angle_rad, nr_stddev, nr_bias_stddev, intensity_lower_threshold_percent,
               srp, local_celes_info);
  return ss;
}
