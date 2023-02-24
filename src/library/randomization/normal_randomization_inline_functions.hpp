/**
 * @file normal_randomization_inline_functions.hpp
 * @brief Class to generate random value with normal distribution with Box-Muller method
 * @note Inline functions
 */
#ifndef S2E_LIBRARY_MATH_NORMAL_RANDOMIZATION_INLINE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_NORMAL_RANDOMIZATION_INLINE_FUNCTIONS_HPP_

namespace libra {

double NormalRand::average() const { return average_; }

void NormalRand::average(double average) { average_ = average; }

double NormalRand::standard_deviation() const { return standard_deviation_; }

void NormalRand::standard_deviation(double standard_deviation) { standard_deviation_ = standard_deviation; }

void NormalRand::set_param(double average, double standard_deviation) {
  average_ = average;
  standard_deviation_ = standard_deviation;
}

void NormalRand::set_param(double average, double standard_deviation, long seed) {
  average_ = average;
  standard_deviation_ = standard_deviation;
  randomizer_.InitSeed(seed);
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_NORMAL_RANDOMIZATION_INLINE_FUNCTIONS_HPP_
