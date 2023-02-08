/**
 * @file GlobalRand.h
 * @brief Class to manage global randomization
 */

#ifndef GLOBALRAND_HPP_
#define GLOBALRAND_HPP_
#include "./Ran0.hpp"

/**
 * @class GlobalRand.h
 * @brief Class to manage global randomization
 * @note Used to make randomized seed for other randomization
 */
class GlobalRand {
 public:
  /**
   * @fn GlobalRand
   * @brief Constructor
   */
  GlobalRand();
  /**
   * @fn SetSeed
   * @brief Set randomized seed value
   */
  void SetSeed(long seed);
  /**
   * @fn MakeSeed
   * @brief Set randomized seed value
   */
  long MakeSeed();

 private:
  static const unsigned int MAX_SEED = 0xffffffff;  //!< Maximum value of seed
  libra::Ran0 base_rand_;                           //!< Base of global randomization
  long seed_;                                       //!< Seed of global randomization
};

extern GlobalRand g_rand;  //!< Global randomization

#endif