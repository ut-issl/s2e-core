/**
 * @file initialize_star_sensor.cpp
 * @brief Initialize functions for star sensor
 */
#include "initialize_star_sensor.hpp"

#include <library/math/constants.hpp>

#include "library/initialize/initialize_file_access.hpp"

using namespace std;

StarSensor InitSTT(ClockGenerator* clock_generator, int sensor_id, const string fname, double compo_step_time, const Dynamics* dynamics,
                   const LocalEnvironment* local_environment) {
  IniAccess STT_conf(fname);
  const char* sensor_name = "STAR_SENSOR_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* STTSection = section_name.c_str();

  int prescaler = STT_conf.ReadInt(STTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  double step_time_s = compo_step_time * prescaler;
  Quaternion quaternion_b2c;
  STT_conf.ReadQuaternion(STTSection, "quaternion_b2c", quaternion_b2c);
  double standard_deviation_orthogonal_direction = STT_conf.ReadDouble(STTSection, "standard_deviation_orthogonal_direction_rad");
  double standard_deviation_sight_direction = STT_conf.ReadDouble(STTSection, "standard_deviation_sight_direction_rad");
  double output_delay_sec = STT_conf.ReadDouble(STTSection, "output_delay_s");
  int output_delay = max(int(output_delay_sec / step_time_s), 1);
  double output_interval_sec = STT_conf.ReadDouble(STTSection, "output_interval_s");
  int output_interval = max(int(output_interval_sec / step_time_s), 1);
  double sun_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "sun_exclusion_angle_deg");
  double sun_forbidden_angle_rad = sun_forbidden_angle_deg * libra::pi / 180.0;
  double earth_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "earth_exclusion_angle_deg");
  double earth_forbidden_angle_rad = earth_forbidden_angle_deg * libra::pi / 180.0;
  double moon_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "moon_exclusion_angle_deg");
  double moon_forbidden_angle_rad = moon_forbidden_angle_deg * libra::pi / 180.0;
  double capture_rate_deg_s = STT_conf.ReadDouble(STTSection, "angular_rate_limit_deg_s");
  double capture_rate_rad_s = capture_rate_deg_s * libra::pi / 180.0;

  StarSensor stt(prescaler, clock_generator, sensor_id, quaternion_b2c, standard_deviation_orthogonal_direction, standard_deviation_sight_direction,
                 step_time_s, output_delay, output_interval, sun_forbidden_angle_rad, earth_forbidden_angle_rad, moon_forbidden_angle_rad,
                 capture_rate_rad_s, dynamics, local_environment);
  return stt;
}

StarSensor InitSTT(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const string fname, double compo_step_time,
                   const Dynamics* dynamics, const LocalEnvironment* local_environment) {
  IniAccess STT_conf(fname);
  const char* sensor_name = "STAR_SENSOR_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* STTSection = section_name.c_str();

  int prescaler = STT_conf.ReadInt(STTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  double step_time_s = compo_step_time * prescaler;

  Quaternion quaternion_b2c;
  STT_conf.ReadQuaternion(STTSection, "quaternion_b2c", quaternion_b2c);
  double standard_deviation_orthogonal_direction = STT_conf.ReadDouble(STTSection, "standard_deviation_orthogonal_direction_rad");
  double standard_deviation_sight_direction = STT_conf.ReadDouble(STTSection, "standard_deviation_sight_direction_rad");
  double output_delay_sec = STT_conf.ReadDouble(STTSection, "output_delay_s");
  int output_delay = max(int(output_delay_sec / step_time_s), 1);
  double output_interval_sec = STT_conf.ReadDouble(STTSection, "output_interval_s");
  int output_interval = max(int(output_interval_sec / step_time_s), 1);
  double sun_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "sun_exclusion_angle_deg");
  double sun_forbidden_angle_rad = sun_forbidden_angle_deg * libra::pi / 180.0;
  double earth_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "earth_exclusion_angle_deg");
  double earth_forbidden_angle_rad = earth_forbidden_angle_deg * libra::pi / 180.0;
  double moon_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "moon_exclusion_angle_deg");
  double moon_forbidden_angle_rad = moon_forbidden_angle_deg * libra::pi / 180.0;
  double capture_rate_deg_s = STT_conf.ReadDouble(STTSection, "angular_rate_limit_deg_s");
  double capture_rate_rad_s = capture_rate_deg_s * libra::pi / 180.0;

  power_port->InitializeWithInitializeFile(fname);

  StarSensor stt(prescaler, clock_generator, power_port, sensor_id, quaternion_b2c, standard_deviation_orthogonal_direction,
                 standard_deviation_sight_direction, step_time_s, output_delay, output_interval, sun_forbidden_angle_rad, earth_forbidden_angle_rad,
                 moon_forbidden_angle_rad, capture_rate_rad_s, dynamics, local_environment);
  return stt;
}
