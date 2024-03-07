/**
 * @file normal_randomization.cpp
 * @brief Class to generate random value with normal distribution with Box-Muller method
 * @note Ref: NUMERICAL RECIPES in C, p.216-p.217
 */
#include "normal_randomization.hpp"
using libra::NormalRand;

#include <cfloat>  //DBL_EPSILON
#include <cmath>   //sqrt, log;

NormalRand::NormalRand() : average_(0.0), standard_deviation_(1.0), holder_(0.0), is_empty_(true) {}

NormalRand::NormalRand(double average, double standard_deviation)
    : average_(average), standard_deviation_(standard_deviation), holder_(0.0), is_empty_(true) {}

NormalRand::NormalRand(double average, double standard_deviation, long seed) throw()
    : average_(average), standard_deviation_(standard_deviation), randomizer_(seed), holder_(0.0), is_empty_(true) {}

NormalRand::operator double() {
  if (is_empty_) {
    double v1, v2, rsq;
    do {
      v1 = 2.0 * double(randomizer_) - 1.0;
      v2 = 2.0 * randomizer_ - 1.0;
      rsq = v1 * v1 + v2 * v2;
    } while (rsq >= 1.0 || rsq < DBL_EPSILON);
    double fac = std::sqrt(-2.0 * std::log(rsq) / rsq);

    holder_ = v1 * fac;
    is_empty_ = false;

    return v2 * fac * standard_deviation_ + average_;
  } else {
    is_empty_ = true;
    return holder_ * standard_deviation_ + average_;
  }
}
