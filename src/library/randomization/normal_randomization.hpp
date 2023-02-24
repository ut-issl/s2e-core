/**
 * @file normal_randomization.hpp
 * @brief Class to generate random value with normal distribution with Box-Muller method
 * @note Ref: NUMERICAL RECIPES in C, p.216-p.217
 */

#ifndef S2E_LIBRARY_RANDOMIZATION_NORMAL_RANDOMIZATION_HPP_
#define S2E_LIBRARY_RANDOMIZATION_NORMAL_RANDOMIZATION_HPP_

#include "minimal_standard_linear_congruential_generator_with_shuffle.hpp"
using libra::MinimalStandardLcgWithShuffle;

namespace libra {

/**
 * @class NormalRand
 * @brief Class to generate random value with normal distribution with Box-Muller method
 */
class NormalRand {
 public:
  /**
   * @fn NormalRand
   * @brief Default constructor initialized as zero average, 1.0 standard deviation
   * @note Used default seed
   */
  NormalRand();

  /**
   * @fn NormalRand
   * @brief Constructor
   * @param avg: Average of normal distribution
   * @param stddev: Standard deviation of normal distribution
   */
  NormalRand(double avg, double stddev);

  /**
   * @fn NormalRand
   * @brief Constructor
   * @param avg: Average of normal distribution
   * @param stddev: Standard deviation of normal distribution
   * @param seed: Seed of randomization
   */
  NormalRand(double avg, double stddev, long seed) throw();

  /**
   * @fn Cast operator to double type
   * @brief Generate random value with the Box-Muller method
   * @return Randomized value
   */
  operator double();

  /**
   * @fn avg
   * @brief Return average
   */
  inline double avg() const;

  /**
   * @fn avg
   * @brief Set average
   */
  inline void avg(double avg);

  /**
   * @fn stddev
   * @brief Return standard deviation
   */
  inline double stddev() const;

  /**
   * @fn stddev
   * @brief Set standard deviation
   */
  inline void stddev(double stddev);

  /**
   * @fn set_param
   * @brief Set parameters
   * @param avg: Average of normal distribution
   * @param stddev: Standard deviation of normal distribution
   */
  inline void set_param(double avg, double stddev);
  /**
   * @fn set_param
   * @brief Set parameters
   * @param avg: Average of normal distribution
   * @param stddev: Standard deviation of normal distribution
   * @param seed: Seed of randomization
   */
  inline void set_param(double avg, double stddev, long seed);

 private:
  double average_;                            //!< Average
  double standard_deviation_;                 //!< Standard deviation
  MinimalStandardLcgWithShuffle randomizer_;  //!< Randomized origin to use Box-Muller method
  double holder_;                             //!< Second random value. Box-Muller method generates two value at once.
                                              //!< The second value is stored and used in the next call.
                                              //!< It means that Box-Muller method is executed once per two call
  bool is_empty_;                             //!< Flag to show the holder_ has available value
};

}  // namespace libra

#include "normal_randomization_inline_functions.hpp"

#endif  // S2E_LIBRARY_RANDOMIZATION_NORMAL_RANDOMIZATION_HPP_
