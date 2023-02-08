/**
 * @file global_randomization.hpp
 * @brief Class to manage global randomization
 */

#ifndef S2E_LIBRARY_MATH_GLOBAL_RANDOMIZATION_H_
#define S2E_LIBRARY_MATH_GLOBAL_RANDOMIZATION_H_

#include "./Ran0.hpp"

/**
 * @class global_randomization.hpp
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

#endif  // S2E_LIBRARY_MATH_GLOBAL_RANDOMIZATION_H_
