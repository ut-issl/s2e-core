/**
 * @class GlobalRand.cpp
 * @brief Class to manage global randomization
 */

#include "GlobalRand.h"

GlobalRand g_rand;

GlobalRand::GlobalRand() { seed_ = 0xdeadbeef; }

void GlobalRand::SetSeed(long seed) {
  base_rand_.init(seed);
  // double dl = base_rand_;
}

long GlobalRand::MakeSeed() {
  double rand = base_rand_;
  long seed = (long)((rand - 0.5) * MAX_SEED);
  if (seed == 0) {
    seed = 0xdeadbeef;
  }
  return seed;
}