#ifndef GLOBALRAND_HPP_
#define GLOBALRAND_HPP_
#include "./Ran0.hpp"

class GlobalRand {
 public:
  GlobalRand();
  void SetSeed(long seed);
  long MakeSeed();

 private:
  static const unsigned int MAX_SEED = 0xffffffff;
  libra::Ran0 base_rand_;
  long seed_;
};

extern GlobalRand g_rand;

#endif