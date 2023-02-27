/**
 * @file sensor_template_functions.hpp
 * @brief Base class for sensor emulation to add noises
 */

#ifndef S2E_COMPONENTS_BASE_SENSOR_TEMPLATE_FUNCTIONS_HPP_
#define S2E_COMPONENTS_BASE_SENSOR_TEMPLATE_FUNCTIONS_HPP_

#include <library/randomization/global_randomization.hpp>

template <size_t N>
Sensor<N>::Sensor(const libra::Matrix<N, N>& scale_factor, const libra::Vector<N>& range_to_const_c, const libra::Vector<N>& range_to_zero_c,
                  const libra::Vector<N>& bias_c, const libra::Vector<N>& normal_random_standard_deviation_c, const double random_walk_step_width_s,
                  const libra::Vector<N>& random_walk_standard_deviation_c, const libra::Vector<N>& random_walk_limit_c)
    : scale_factor_(scale_factor),
      range_to_const_c_(range_to_const_c),
      range_to_zero_c_(range_to_zero_c),
      bias_c_(bias_c),
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
  calc_value_c += bias_c_;
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

#endif  // S2E_COMPONENTS_BASE_SENSOR_TEMPLATE_FUNCTIONS_HPP_
