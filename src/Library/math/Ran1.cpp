/**
 * @file Ran1.cpp
 * @brief Randomization with Park and Miller's multiplicative congruential method combined with mixed method
 * @note ran1 function in "NUMERICAL RECIPES in C, p.207-208"
 */

#include "Ran1.hpp"
using libra::Ran1;

Ran1::Ran1() : y_(0) { init_(); }

Ran1::Ran1(long seed) : ran0_(seed), y_(0) { init_(); }

void Ran1::init_seed(long seed) {
  ran0_.init(seed);
  init_();
}

void Ran1::init_() {
  // Warmup of ran0_
  for (int i = 0; i < 8; i++) {
    double temp = ran0_;
    static_cast<void>(temp);
  }
  // Fill mixing table
  for (size_t i = 0; i < V_SIZE_; i++) {
    vec_[i] = ran0_;
  }
  //    for(size_t i=0; i<V_SIZE_; i++){ v_[i] = ran0_; }
}

Ran1::operator double() {
  double out = vec_[y_];
  vec_[y_] = ran0_;  // Compensate next random value
  y_ = (size_t)out * Ran0::M;
  y_ %= V_SIZE_;  // y <- [0 : V_SIZE_-1]

  return out;
}
