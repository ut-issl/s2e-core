/**
 * @file random_walk_template_functions.hpp
 * @brief Class to calculate random wark value (template functions)
 */

#ifndef S2E_LIBRARY_RANDOMIZATION_RANDOM_WALK_TEMPLATE_FUNCTIONS_HPP_
#define S2E_LIBRARY_RANDOMIZATION_RANDOM_WALK_TEMPLATE_FUNCTIONS_HPP_

#include <math_physics/randomization/global_randomization.hpp>
#include <utilities/macros.hpp>

template <size_t N>
RandomWalk<N>::RandomWalk(double step_width_s, const math::Vector<N>& standard_deviation, const math::Vector<N>& limit)
    : math::OrdinaryDifferentialEquation<N>(step_width_s), limit_(limit) {
  // Set standard deviation
  for (size_t i = 0; i < N; ++i) {
    normal_randomizer_[i].SetParameters(0.0, standard_deviation[i], s2e::randomization::global_randomization.MakeSeed());
  }
}

template <size_t N>
void RandomWalk<N>::DerivativeFunction(double x, const math::Vector<N>& state, math::Vector<N>& rhs) {
  UNUSED(x);  // TODO: consider the x is really need for this function

  for (size_t i = 0; i < N; ++i) {
    if (state[i] > limit_[i])
      rhs[i] = -fabs(normal_randomizer_[i]);
    else if (state[i] < -limit_[i])
      rhs[i] = fabs(normal_randomizer_[i]);
    else
      rhs[i] = normal_randomizer_[i];
  }
}

#endif  // S2E_LIBRARY_RANDOMIZATION_RANDOM_WALK_TEMPLATE_FUNCTIONS_HPP_
