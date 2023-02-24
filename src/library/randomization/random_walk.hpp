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
class RandomWalk : public libra::ODE<N> {
 public:
  /**
   * @fn RandomWalk
   * @brief Constructor
   * @param step_width: Step width
   * @param standard_deviation: Standard deviation of random walk excitation noise
   * @param limit: Limit of random walk
   */
  RandomWalk(double step_width, const libra::Vector<N>& standard_deviation, const libra::Vector<N>& limit);

  /**
   * @fn RHS
   * @brief Override function of ODE to define the difference equation
   * @param [in] x: Independent variable (e.g. time)
   * @param [in] state: State vector
   * @param [out] rhs: Differentiated value of state vector
   */
  virtual void RHS(double x, const libra::Vector<N>& state, libra::Vector<N>& rhs);

 private:
  libra::Vector<N> limit_;                  //!< Limit of random walk
  libra::NormalRand normal_randomizer_[N];  //!< Random walk excitation noise
};

#include "random_walk_template_functions.hpp"  // template function definisions.

#endif  // S2E_LIBRARY_RANDOMIZATION_RANDOM_WALK_HPP_