
#pragma once
#include <Interface/InitInput/IniAccess.h>

template <size_t N>
SensorBase<N> ReadSensorBaseInformation(const std::string file_name, const double step_width_s) {
  IniAccess ini_file(file_name);
  char section[30] = "SensorBase";

  libra::Vector<N * N> scale_factor_vector;
  ini_file.ReadVector(section, "scale_factor_c", scale_factor_vector);
  libra::Matrix<N, N> scale_factor_c;
  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      scale_factor_c[i][j] = scale_factor_vector[i * N + j];
    }
  }

  libra::Vector<N> constant_bias_c;
  ini_file.ReadVector(section, "constant_bias_c", constant_bias_c);
  libra::Vector<N> normal_random_standard_deviation_c;
  ini_file.ReadVector(section, "normal_random_standard_deviation_c", normal_random_standard_deviation_c);
  libra::Vector<N> random_walk_standard_deviation_c;
  ini_file.ReadVector(section, "random_walk_standard_deviation_c", random_walk_standard_deviation_c);
  libra::Vector<N> random_walk_limit_c;
  ini_file.ReadVector(section, "random_walk_limit_c", random_walk_limit_c);

  double range_to_const = ini_file.ReadDouble(section, "range_to_const");
  libra::Vector<N> range_to_const_c{range_to_const};
  double range_to_zero = ini_file.ReadDouble(section, "range_to_zero");
  libra::Vector<N> range_to_zero_c{range_to_zero};

  SensorBase<N> sensor_base(scale_factor_c, range_to_const_c, range_to_zero_c, constant_bias_c, normal_random_standard_deviation_c, step_width_s,
                            random_walk_standard_deviation_c, random_walk_limit_c);

  return sensor_base;
}
