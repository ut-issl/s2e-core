/**
 * @file global_randomization.hpp
 * @brief Class to manage global randomization
 */

#ifndef S2E_LIBRARY_RANDOMIZATION_GLOBAL_RANDOMIZATION_HPP_
#define S2E_LIBRARY_RANDOMIZATION_GLOBAL_RANDOMIZATION_HPP_

#include "./minimal_standard_linear_congruential_generator.hpp"

/**
 * @class global_randomization.hpp
 * @brief Class to manage global randomization
 * @note Used to make randomized seed for other randomization
 */
class GlobalRandomization {
 public:
  /**
   * @fn GlobalRandomization
   * @brief Constructor
   */
  GlobalRandomization();
  /**
   * @fn SetSeed
   * @brief Set randomized seed value
   */
  void SetSeed(const long seed);
  /**
   * @fn MakeSeed
   * @brief Set randomized seed value
   */
  long MakeSeed();

 private:
  static const unsigned int kMaxSeed = 0xffffffff;  //!< Maximum value of seed
  randomization::MinimalStandardLcg base_randomizer_;       //!< Base of global randomization
  long seed_;                                       //!< Seed of global randomization
};

extern GlobalRandomization global_randomization;  //!< Global randomization

#endif  // S2E_LIBRARY_RANDOMIZATION_GLOBAL_RANDOMIZATION_HPP_
