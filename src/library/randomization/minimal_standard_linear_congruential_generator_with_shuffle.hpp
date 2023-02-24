/**
 * @file minimal_standard_linear_congruential_generator_with_shuffle.hpp
 * @brief Randomization with Park and Miller's multiplicative congruential method combined with mixed method
 * @note ran1 function in "NUMERICAL RECIPES in C, p.207-208"
 */

#ifndef S2E_LIBRARY_RANDOMIZATION_MINIMAL_STANDARD_LINEAR_CONGRUENTIAL_GENERATOR_WITH_SHUFFLE_HPP_
#define S2E_LIBRARY_RANDOMIZATION_MINIMAL_STANDARD_LINEAR_CONGRUENTIAL_GENERATOR_WITH_SHUFFLE_HPP_

#include <cstddef>  // size_t

#include "minimal_standard_linear_congruential_generator.hpp"

namespace libra {

/**
 * @class MinimalStandardLcgWithShuffle
 * @brief Randomization with Park and Miller's multiplicative congruential method combined with mixed method
 */
class MinimalStandardLcgWithShuffle {
 public:
  /**
   * @fn MinimalStandardLcgWithShuffle
   * @brief Default constructor with default seed value
   */
  MinimalStandardLcgWithShuffle();
  /**
   * @fn MinimalStandardLcgWithShuffle
   * @brief Default constructor with seed value
   * @param [in] seed: Seed of randomization
   */
  explicit MinimalStandardLcgWithShuffle(const long seed);

  /**
   * @fn Cast operator of double type
   * @brief Generate randomized value when casting
   * @return Generated randomized value
   */
  operator double();

  /**
   * @fn InitSeed
   * @brief Set seed value
   * @param [in] seed: Seed of randomization
   */
  void InitSeed(const long seed);

 private:
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize();

  MinimalStandardLcg minimal_lcg_;           //!< Randomization with Park and Miller's multiplicative congruential method
  static const std::size_t kTableSize = 32;  //!< Number of elements for mixing table
  std::size_t table_position_;               //!< Position of mixing table
  double mixing_table_[kTableSize];          //!< Mixing table
};

}  // namespace libra

#endif  // S2E_LIBRARY_RANDOMIZATION_MINIMAL_STANDARD_LINEAR_CONGRUENTIAL_GENERATOR_WITH_SHUFFLE_HPP_
