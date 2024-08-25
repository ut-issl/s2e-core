/**
 * @file randomize1.cpp
 * @brief Randomization with Park and Miller's multiplicative congruential method combined with mixed method
 * @note ran1 function in "NUMERICAL RECIPES in C, p.207-208"
 */

#include "minimal_standard_linear_congruential_generator_with_shuffle.hpp"
using randomization::MinimalStandardLcgWithShuffle;

MinimalStandardLcgWithShuffle::MinimalStandardLcgWithShuffle() : table_position_(0) { Initialize(); }

MinimalStandardLcgWithShuffle::MinimalStandardLcgWithShuffle(const long seed) : minimal_lcg_(seed), table_position_(0) { Initialize(); }

void MinimalStandardLcgWithShuffle::InitSeed(const long seed) {
  minimal_lcg_.Initialize(seed);
  Initialize();
}

void MinimalStandardLcgWithShuffle::Initialize() {
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

MinimalStandardLcgWithShuffle::operator double() {
  double out = mixing_table_[table_position_];
  mixing_table_[table_position_] = minimal_lcg_;  // Compensate next random value
  table_position_ = (size_t)out * MinimalStandardLcg::kM;
  table_position_ %= kTableSize;  // y <- [0 : kTableSize-1]

  return out;
}
