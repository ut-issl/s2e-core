/**
 * @file randomize1.cpp
 * @brief Randomization with Park and Miller's multiplicative congruential method combined with mixed method
 * @note ran1 function in "NUMERICAL RECIPES in C, p.207-208"
 */

#include "minimal_standard_linear_congruential_generator_with_shuffle.hpp"
using libra::Ran1;

Ran1::Ran1() : table_position_(0) { Initialize(); }

Ran1::Ran1(const long seed) : minimal_lcg_(seed), table_position_(0) { Initialize(); }

void Ran1::InitSeed(const long seed) {
  minimal_lcg_.Initialize(seed);
  Initialize();
}

void Ran1::Initialize() {
  // Warmup of minimal_lcg_
  for (int i = 0; i < 8; i++) {
    double temp = minimal_lcg_;
    static_cast<void>(temp);
  }
  // Fill mixing table
  for (size_t i = 0; i < kTableSize; i++) {
    mixing_table_[i] = minimal_lcg_;
  }
}

Ran1::operator double() {
  double out = mixing_table_[table_position_];
  mixing_table_[table_position_] = minimal_lcg_;  // Compensate next random value
  table_position_ = (size_t)out * Ran0::kM;
  table_position_ %= kTableSize;  // y <- [0 : kTableSize-1]

  return out;
}
