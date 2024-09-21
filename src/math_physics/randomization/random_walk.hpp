/**
 * @file random_walk.hpp
 * @brief Class to calculate random wark value
 */

#ifndef S2E_LIBRARY_RANDOMIZATION_RANDOM_WALK_HPP_
#define S2E_LIBRARY_RANDOMIZATION_RANDOM_WALK_HPP_

#include "../math/ordinary_differential_equation.hpp"
#include "../math/vector.hpp"
#include "./normal_randomization.hpp"

/**
 * @class RandomWalk
 * @brief Class to calculate random wark value
 */
template <size_t N>
class RandomWalk : public math::OrdinaryDifferentialEquation<N> {
 public:
  /**
   * @fn RandomWalk
   * @brief Constructor
   * @param step_width_s: Step width
   * @param standard_deviation: Standard deviation of random walk excitation noise
   * @param limit: Limit of random walk
   */
  RandomWalk(double step_width_s, const math::Vector<N>& standard_deviation, const math::Vector<N>& limit);

  /**
   * @fn DerivativeFunction
   * @brief Override function of OrdinaryDifferentialEquation to define the difference equation
   * @param [in] x: Independent variable (e.g. time)
   * @param [in] state: State vector
   * @param [out] rhs: Differentiated value of state vector
   */
  virtual void DerivativeFunction(double x, const math::Vector<N>& state, math::Vector<N>& rhs);

 private:
  math::Vector<N> limit_;                           //!< Limit of random walk
  randomization::NormalRand normal_randomizer_[N];  //!< Random walk excitation noise
};

#include "random_walk_template_functions.hpp"  // template function definisions.

#endif  // S2E_LIBRARY_RANDOMIZATION_RANDOM_WALK_HPP_