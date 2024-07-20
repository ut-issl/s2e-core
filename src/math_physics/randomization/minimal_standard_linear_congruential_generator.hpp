/**
 * @file minimal_standard_linear_congruential_generator.hpp
 * @brief Randomization with Park and Miller's multiplicative congruential method
 * @note ran0 function in "NUMERICAL RECIPES in C, p.206"
 */

#ifndef S2E_LIBRARY_RANDOMIZATION_MINIMAL_STANDARD_LINEAR_CONGRUENTIAL_GENERATOR_HPP_
#define S2E_LIBRARY_RANDOMIZATION_MINIMAL_STANDARD_LINEAR_CONGRUENTIAL_GENERATOR_HPP_

namespace libra {

/**
 * @class MinimalStandardLcg
 * @brief Randomization with Park and Miller's multiplicative congruential and mixed method
 */
class MinimalStandardLcg {
 public:
  static const long kA = 16807;       //!< Coefficient a for multiplication
  static const long kM = 2147483647;  //!< Divisor for modulo

  /**
   * @fn MinimalStandardLcg
   * @brief Default constructor with default seed value
   */
  MinimalStandardLcg();
  /**
   * @fn MinimalStandardLcg
   * @brief Default constructor with seed value
   * @param [in] seed: Seed of randomization
   */
  explicit MinimalStandardLcg(const long seed);

  /**
   * @fn Initialize
   * @brief Set seed value
   * @param [in] seed: Seed of randomization
   */
  void Initialize(const long seed);

  /**
   * @fn Cast operator of double type
   * @brief Generate randomized value when casting
   * @return Generated randomized value
   */
  operator double();

 private:
  static const double a_m_;       //!< A/M
  static const long q_ = 127773;  //!< Integer part of A/M
  static const long r_ = 2836;    //!< m mod a

  long seed_;  //!< Seed of randomization
};

}  // namespace math

#endif  // S2E_LIBRARY_RANDOMIZATION_MINIMAL_STANDARD_LINEAR_CONGRUENTIAL_GENERATOR_HPP_
