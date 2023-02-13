/**
 * @file normal_randomization_inline_functions.hpp
 * @brief Class to generate random value with normal distribution with Box-Muller method
 * @note Inline functions
 */
#ifndef S2E_LIBRARY_MATH_NORMAL_RANDOMIZATION_INLINE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_NORMAL_RANDOMIZATION_INLINE_FUNCTIONS_HPP_

namespace libra {

double NormalRand::avg() const { return avg_; }

void NormalRand::avg(double avg) { avg_ = avg; }

double NormalRand::stddev() const { return stddev_; }

void NormalRand::stddev(double stddev) { stddev_ = stddev; }

void NormalRand::set_param(double avg, double stddev) {
  avg_ = avg;
  stddev_ = stddev;
}

void NormalRand::set_param(double avg, double stddev, long seed) {
  avg_ = avg;
  stddev_ = stddev;
  rand_.init_seed(seed);
}

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_NORMAL_RANDOMIZATION_INLINE_FUNCTIONS_HPP_
