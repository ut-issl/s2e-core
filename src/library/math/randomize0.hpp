/**
 * @file randomize0.hpp
 * @brief Randomization with Park and Miller's multiplicative congruential method
 * @note ran0 function in "NUMERICAL RECIPES in C, p.206"
 */

#ifndef S2E_LIBRARY_MATH_RANDOMIZE0_H_
#define S2E_LIBRARY_MATH_RANDOMIZE0_H_

namespace libra {

/**
 * @class Ran0
 * @brief Randomization with Park and Miller's multiplicative congruential and mixed method
 */
class Ran0 {
 public:
  static const long A = 16807;       //!< Coefficient a for multiplication
  static const long M = 2147483647;  //!< Divisor for modulo

  /**
   * @fn Ran0
   * @brief Default constructor with default seed value
   */
  Ran0();
  /**
   * @fn Ran0
   * @brief Default constructor with seed value
   * @param [in] seed: Seed of randomization
   */
  explicit Ran0(long seed);

  /**
   * @fn init
   * @brief Set seed value
   * @param [in] seed: Seed of randomization
   */
  void init(long seed);

  /**
   * @fn Cast operator of double type
   * @brief Generate randomized value when casting
   * @return Generated randomized value
   */
  operator double();

 private:
  static const double AM_;        //!< A/M
  static const long Q_ = 127773;  //!< Integer part of A/M
  static const long R_ = 2836;    //!< m mod a

  long seed_;  //!< Seed of randomization
};

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_RANDOMIZE0_H_
