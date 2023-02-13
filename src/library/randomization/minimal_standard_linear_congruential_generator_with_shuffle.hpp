/**
 * @file minimal_standard_linear_congruential_generator_with_shuffle.hpp
 * @brief Randomization with Park and Miller's multiplicative congruential method combined with mixed method
 * @note ran1 function in "NUMERICAL RECIPES in C, p.207-208"
 */

#ifndef S2E_LIBRARY_MATH_RANDOMIZE1_HPP_
#define S2E_LIBRARY_MATH_RANDOMIZE1_HPP_

#include <cstddef>  // size_t

#include "minimal_standard_linear_congruential_generator.hpp"

namespace libra {

/**
 * @class Ran1
 * @brief Randomization with Park and Miller's multiplicative congruential method combined with mixed method
 */
class Ran1 {
 public:
  /**
   * @fn Ran1
   * @brief Default constructor with default seed value
   */
  Ran1();
  /**
   * @fn Ran1
   * @brief Default constructor with seed value
   * @param [in] seed: Seed of randomization
   */
  explicit Ran1(long seed);

  /**
   * @fn Cast operator of double type
   * @brief Generate randomized value when casting
   * @return Generated randomized value
   */
  operator double();

  /**
   * @fn init_seed
   * @brief Set seed value
   * @param [in] seed: Seed of randomization
   */
  void init_seed(long seed);

 private:
  /**
   * @fn init_
   * @brief Initialize function
   */
  void init_();

  Ran0 ran0_;                             //!< Randomization with Park and Miller's multiplicative congruential method
  static const std::size_t V_SIZE_ = 32;  //!< Number of elements for mixing table
  double v_[V_SIZE_];                     //!< Mixing table (Not used now)
  std::size_t y_;                         //!< Position of mixing table
  double vec_[V_SIZE_];                   //!< Mixing table
};

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_RANDOMIZE1_HPP_
