/**
 * @file randomize0.cpp
 * @brief Randomization with Park and Miller's multiplicative congruential method
 * @note ran0 function in "NUMERICAL RECIPES in C, p.206"
 */

#include "minimal_standard_linear_congruential_generator.hpp"
using libra::Ran0;

#include <stdexcept>

const double Ran0::a_m_ = 1.0 / Ran0::kM;

Ran0::Ran0() : seed_(0xdeadbeef) {}

Ran0::Ran0(const long seed) : seed_(seed) {
  if (seed == 0) {
    throw std::invalid_argument("Ran0:: seed is 0.");
  }
}
void Ran0::Initialize(const long seed) {
  if (seed == 0) {
    throw std::invalid_argument("Ran0:: seed is 0.");
  }
  seed_ = seed;
}

Ran0::operator double() {
  long k = seed_ / q_;
  seed_ = kA * (seed_ - k * q_) - r_ * k;
  if (seed_ < 0) {
    seed_ += kM;
  }
  return a_m_ * seed_;
}
