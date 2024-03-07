/**
 * @file sensor_template_functions.hpp
 * @brief Base class for sensor emulation to add noises
 */

#ifndef S2E_COMPONENTS_BASE_SENSOR_TEMPLATE_FUNCTIONS_HPP_
#define S2E_COMPONENTS_BASE_SENSOR_TEMPLATE_FUNCTIONS_HPP_

#include <math_physics/randomization/global_randomization.hpp>
#include <setting_file_reader/initialize_file_access.hpp>

template <size_t N>
Sensor<N>::Sensor(const libra::Matrix<N, N>& scale_factor, const libra::Vector<N>& range_to_const_c, const libra::Vector<N>& range_to_zero_c,
                  const libra::Vector<N>& bias_noise_c, const libra::Vector<N>& normal_random_standard_deviation_c,
                  const double random_walk_step_width_s, const libra::Vector<N>& random_walk_standard_deviation_c,
                  const libra::Vector<N>& random_walk_limit_c)
    : bias_noise_c_(bias_noise_c),
      scale_factor_(scale_factor),
      range_to_const_c_(range_to_const_c),
      range_to_zero_c_(range_to_zero_c),
      random_walk_noise_c_(random_walk_step_width_s, random_walk_standard_deviation_c, random_walk_limit_c) {
  for (size_t i = 0; i < N; i++) {
    normal_random_noise_c_[i].SetParameters(0.0, normal_random_standard_deviation_c[i], global_randomization.MakeSeed());
  }
  RangeCheck();
}

template <size_t N>
Sensor<N>::~Sensor() {}

template <size_t N>
libra::Vector<N> Sensor<N>::Measure(const libra::Vector<N> true_value_c) {
  libra::Vector<N> calc_value_c;
  calc_value_c = scale_factor_ * true_value_c;
  calc_value_c += bias_noise_c_;
  for (size_t i = 0; i < N; ++i) {
    calc_value_c[i] += random_walk_noise_c_[i];
    calc_value_c[i] += normal_random_noise_c_[i];
  }
  ++random_walk_noise_c_;  // update Random Walk
  return Clip(calc_value_c);
}

template <size_t N>
libra::Vector<N> Sensor<N>::Clip(const libra::Vector<N> input_c) {
  libra::Vector<N> output_c;
  for (size_t i = 0; i < N; ++i) {
    if (input_c[i] >= range_to_const_c_[i] && input_c[i] < range_to_zero_c_[i]) {
      output_c[i] = range_to_const_c_[i];
    } else if (input_c[i] <= -range_to_const_c_[i] && input_c[i] > -range_to_zero_c_[i]) {
      output_c[i] = -range_to_const_c_[i];
    } else if (fabs(input_c[i]) >= range_to_zero_c_[i]) {
      output_c[i] = 0.0;
    } else {
      output_c[i] = input_c[i];
    }
  }
  return output_c;
}

template <size_t N>
void Sensor<N>::RangeCheck(void) {
  for (size_t i = 0; i < N; i++) {
    if (range_to_const_c_[i] < 0.0 || range_to_zero_c_[i] < 0.0) {
      std::cout << "Sensor: Range should be positive!!\n";
      std::cout << "The range values are set as positive.\n";
      range_to_zero_c_[i] = fabs(range_to_zero_c_[i]);
      range_to_const_c_[i] = fabs(range_to_const_c_[i]);
    }
    if (range_to_const_c_[i] > range_to_zero_c_[i]) {
      std::cout << "Sensor: range_zero should be greater than range_const!!\n";
      std::cout << "The range_zero is set as twice value of the range_const.\n";
      range_to_zero_c_[i] = 2.0 * range_to_const_c_[i];
    }
  }
}

template <size_t N>
Sensor<N> ReadSensorInformation(const std::string file_name, const double step_width_s, const std::string component_name, const std::string unit) {
  IniAccess ini_file(file_name);
  std::string section = "SENSOR_BASE_" + component_name;

  libra::Vector<N * N> scale_factor_vector;
  ini_file.ReadVector(section.c_str(), "scale_factor_c", scale_factor_vector);
  libra::Matrix<N, N> scale_factor_c;
  if (scale_factor_vector.CalcNorm() == 0.0) {
    scale_factor_c = libra::MakeIdentityMatrix<N>();
  } else {
    for (size_t i = 0; i < N; i++) {
      for (size_t j = 0; j < N; j++) {
        scale_factor_c[i][j] = scale_factor_vector[i * N + j];
      }
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

  Sensor<N> sensor_base(scale_factor_c, range_to_const_c, range_to_zero_c, constant_bias_c, normal_random_standard_deviation_c, step_width_s,
                        random_walk_standard_deviation_c, random_walk_limit_c);

  return sensor_base;
}

#endif  // S2E_COMPONENTS_BASE_SENSOR_TEMPLATE_FUNCTIONS_HPP_
