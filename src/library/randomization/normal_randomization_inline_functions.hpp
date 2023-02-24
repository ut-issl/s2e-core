/**
 * @file normal_randomization_inline_functions.hpp
 * @brief Class to generate random value with normal distribution with Box-Muller method
 * @note Inline functions
 */
#ifndef S2E_LIBRARY_MATH_NORMAL_RANDOMIZATION_INLINE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_NORMAL_RANDOMIZATION_INLINE_FUNCTIONS_HPP_

namespace libra {

double NormalRand::avg() const { return average_; }

void NormalRand::avg(double avg) { average_ = avg; }

double NormalRand::stddev() const { return standard_deviation_; }

void NormalRand::stddev(double stddev) { standard_deviation_ = stddev; }

void NormalRand::set_param(double avg, double stddev) {
  average_ = avg;
  standard_deviation_ = stddev;
}

void NormalRand::set_param(double avg, double stddev, long seed) {
  average_ = avg;
  standard_deviation_ = stddev;
  randomizer_.InitSeed(seed);
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_NORMAL_RANDOMIZATION_INLINE_FUNCTIONS_HPP_
