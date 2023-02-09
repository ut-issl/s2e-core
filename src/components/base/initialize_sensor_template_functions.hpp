/**
 * @file initialize_sensor_template_functions.hpp
 * @brief Initialize functions for SensorBase class (template functions)
 */

#ifndef S2E_COMPONENTS_BASE_INITIALIZE_SENSOR_TEMPLATE_FUNCTIONS_HPP_
#define S2E_COMPONENTS_BASE_INITIALIZE_SENSOR_TEMPLATE_FUNCTIONS_HPP_

#include "interface/initialize/initialize_file_access.hpp"

template <size_t N>
SensorBase<N> ReadSensorBaseInformation(const std::string file_name, const double step_width_s, const std::string component_name,
                                        const std::string unit) {
  IniAccess ini_file(file_name);
  std::string section = "SENSOR_BASE_" + component_name;

  libra::Vector<N * N> scale_factor_vector;
  ini_file.ReadVector(section.c_str(), "scale_factor_c", scale_factor_vector);
  libra::Matrix<N, N> scale_factor_c;
  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      scale_factor_c[i][j] = scale_factor_vector[i * N + j];
    }
  }

  std::string key_name;
  libra::Vector<N> constant_bias_c;
  key_name = "constant_bias_c_" + unit;
  ini_file.ReadVector(section.c_str(), key_name.c_str(), constant_bias_c);
  libra::Vector<N> normal_random_standard_deviation_c;
  key_name = "normal_random_standard_deviation_c_" + unit;
  ini_file.ReadVector(section.c_str(), key_name.c_str(), normal_random_standard_deviation_c);
  libra::Vector<N> random_walk_standard_deviation_c;
  key_name = "random_walk_standard_deviation_c_" + unit;
  ini_file.ReadVector(section.c_str(), key_name.c_str(), random_walk_standard_deviation_c);
  libra::Vector<N> random_walk_limit_c;
  key_name = "random_walk_limit_c_" + unit;
  ini_file.ReadVector(section.c_str(), key_name.c_str(), random_walk_limit_c);

  key_name = "range_to_constant_" + unit;
  double range_to_const = ini_file.ReadDouble(section.c_str(), key_name.c_str());
  libra::Vector<N> range_to_const_c{range_to_const};

  key_name = "range_to_zero_" + unit;
  double range_to_zero = ini_file.ReadDouble(section.c_str(), key_name.c_str());
  libra::Vector<N> range_to_zero_c{range_to_zero};

  SensorBase<N> sensor_base(scale_factor_c, range_to_const_c, range_to_zero_c, constant_bias_c, normal_random_standard_deviation_c, step_width_s,
                            random_walk_standard_deviation_c, random_walk_limit_c);

  return sensor_base;
}

#endif  // S2E_COMPONENTS_BASE_INITIALIZE_SENSOR_TEMPLATE_FUNCTIONS_HPP_
