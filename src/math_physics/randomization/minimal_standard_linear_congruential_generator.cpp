/**
 * @file randomize0.cpp
 * @brief Randomization with Park and Miller's multiplicative congruential method
 * @note ran0 function in "NUMERICAL RECIPES in C, p.206"
 */

#include "minimal_standard_linear_congruential_generator.hpp"
using s2e::randomization::MinimalStandardLcg;

#include <stdexcept>

const double MinimalStandardLcg::a_m_ = 1.0 / MinimalStandardLcg::kM;

MinimalStandardLcg::MinimalStandardLcg() : seed_(0xdeadbeef) {}

MinimalStandardLcg::MinimalStandardLcg(const long seed) : seed_(seed) {
  if (seed == 0) {
    throw std::invalid_argument("MinimalStandardLcg:: seed is 0.");
  }
}
void MinimalStandardLcg::Initialize(const long seed) {
  if (seed == 0) {
    throw std::invalid_argument("MinimalStandardLcg:: seed is 0.");
  }
  seed_ = seed;
}

MinimalStandardLcg::operator double() {
  long k = seed_ / q_;
  seed_ = kA * (seed_ - k * q_) - r_ * k;
  if (seed_ < 0) {
    seed_ += kM;
  }
  return a_m_ * seed_;
}
