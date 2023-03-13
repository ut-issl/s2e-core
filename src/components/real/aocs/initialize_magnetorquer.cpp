/**
 * @file initialize_magnetorquer.cpp
 * @brief Initialize functions for magnetorquer
 */
#include "initialize_magnetorquer.hpp"

#include "library/initialize/initialize_file_access.hpp"

Magnetorquer InitMagnetorquer(ClockGenerator* clock_generator, int actuator_id, const std::string file_name, double component_step_time_s,
                              const GeomagneticField* geomagnetic_field) {
  IniAccess magtorquer_conf(file_name);
  const char* sensor_name = "MAGNETORQUER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(actuator_id));
  const char* MTSection = section_name.c_str();

  int prescaler = magtorquer_conf.ReadInt(MTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  libra::Vector<kMtqDimension * kMtqDimension> sf_vec;
  magtorquer_conf.ReadVector(MTSection, "scale_factor_c", sf_vec);
  libra::Matrix<kMtqDimension, kMtqDimension> scale_factor;
  for (size_t i = 0; i < kMtqDimension; i++) {
    for (size_t j = 0; j < kMtqDimension; j++) {
      scale_factor[i][j] = sf_vec[i * kMtqDimension + j];
    }
  }

  libra::Quaternion quaternion_b2c;
  magtorquer_conf.ReadQuaternion(MTSection, "quaternion_b2c", quaternion_b2c);

  libra::Vector<kMtqDimension> max_magnetic_moment_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "max_output_magnetic_moment_c_Am2", max_magnetic_moment_c_Am2);

  libra::Vector<kMtqDimension> min_magnetic_moment_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "min_output_magnetic_moment_c_Am2", min_magnetic_moment_c_Am2);

  libra::Vector<kMtqDimension> bias_noise_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "constant_bias_noise_c_Am2", bias_noise_c_Am2);

  double random_walk_step_width_s = component_step_time_s * (double)prescaler;
  libra::Vector<kMtqDimension> random_walk_standard_deviation_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "random_walk_standard_deviation_c_Am2", random_walk_standard_deviation_c_Am2);
  libra::Vector<kMtqDimension> random_walk_limit_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "random_walk_limit_c_Am2", random_walk_limit_c_Am2);
  libra::Vector<kMtqDimension> normal_random_standard_deviation_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "white_noise_standard_deviation_c_Am2", normal_random_standard_deviation_c_Am2);

  Magnetorquer magtorquer(prescaler, clock_generator, actuator_id, quaternion_b2c, scale_factor, max_magnetic_moment_c_Am2, min_magnetic_moment_c_Am2,
                          bias_noise_c_Am2, random_walk_step_width_s, random_walk_standard_deviation_c_Am2, random_walk_limit_c_Am2,
                          normal_random_standard_deviation_c_Am2, geomagnetic_field);
  return magtorquer;
}

Magnetorquer InitMagnetorquer(ClockGenerator* clock_generator, PowerPort* power_port, int actuator_id, const std::string file_name,
                              double component_step_time_s, const GeomagneticField* geomagnetic_field) {
  IniAccess magtorquer_conf(file_name);
  const char* sensor_name = "MAGNETORQUER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(actuator_id));
  const char* MTSection = section_name.c_str();

  int prescaler = magtorquer_conf.ReadInt(MTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  libra::Vector<kMtqDimension * kMtqDimension> sf_vec;
  magtorquer_conf.ReadVector(MTSection, "scale_factor_c", sf_vec);
  libra::Matrix<kMtqDimension, kMtqDimension> scale_factor;
  for (size_t i = 0; i < kMtqDimension; i++) {
    for (size_t j = 0; j < kMtqDimension; j++) {
      scale_factor[i][j] = sf_vec[i * kMtqDimension + j];
    }
  }

  libra::Quaternion quaternion_b2c;
  magtorquer_conf.ReadQuaternion(MTSection, "quaternion_b2c", quaternion_b2c);

  libra::Vector<kMtqDimension> max_magnetic_moment_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "max_output_magnetic_moment_c_Am2", max_magnetic_moment_c_Am2);

  libra::Vector<kMtqDimension> min_magnetic_moment_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "min_output_magnetic_moment_c_Am2", min_magnetic_moment_c_Am2);

  libra::Vector<kMtqDimension> bias_noise_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "constant_bias_noise_c_Am2", bias_noise_c_Am2);

  double random_walk_step_width_s = component_step_time_s * (double)prescaler;
  libra::Vector<kMtqDimension> random_walk_standard_deviation_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "random_walk_standard_deviation_c_Am2", random_walk_standard_deviation_c_Am2);
  libra::Vector<kMtqDimension> random_walk_limit_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "random_walk_limit_c_Am2", random_walk_limit_c_Am2);
  libra::Vector<kMtqDimension> normal_random_standard_deviation_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "white_noise_standard_deviation_c_Am2", normal_random_standard_deviation_c_Am2);

  // PowerPort
  power_port->InitializeWithInitializeFile(file_name);

  Magnetorquer magtorquer(prescaler, clock_generator, power_port, actuator_id, quaternion_b2c, scale_factor, max_magnetic_moment_c_Am2,
                          min_magnetic_moment_c_Am2, bias_noise_c_Am2, random_walk_step_width_s, random_walk_standard_deviation_c_Am2,
                          random_walk_limit_c_Am2, normal_random_standard_deviation_c_Am2, geomagnetic_field);
  return magtorquer;
}
