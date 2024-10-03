/**
 * @class global_randomization.cpp
 * @brief Class to manage global randomization
 */

#include "global_randomization.hpp"

namespace s2e::randomization {

GlobalRandomization global_randomization;

GlobalRandomization::GlobalRandomization() { seed_ = 0xdeadbeef; }

void GlobalRandomization::SetSeed(const long seed) {
  base_randomizer_.Initialize(seed);
  // double dl = base_randomizer_;
}

long GlobalRandomization::MakeSeed() {
  double rand = base_randomizer_;
  long seed = (long)((rand - 0.5) * kMaxSeed);
  if (seed == 0) {
    seed = 0xdeadbeef;
  }
  return seed;
}

}  // namespace s2e::randomization
