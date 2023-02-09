/**
 * @file random_walk_tfs.hpp
 * @brief Class to calculate random wark value (template functions)
 */

#ifndef S2E_LIBRARY_MATH_RANDOM_WALK_TFS_HPP_
#define S2E_LIBRARY_MATH_RANDOM_WALK_TFS_HPP_

#include <library/math/global_randomization.hpp>
#include <library/utilities/macros.hpp>

template <size_t N>
RandomWalk<N>::RandomWalk(double step_width, const libra::Vector<N>& stddev, const libra::Vector<N>& limit)
    : libra::ODE<N>(step_width), limit_(limit) {
  // Set standard deviation
  for (size_t i = 0; i < N; ++i) {
    nrs_[i].set_param(0.0, stddev[i], g_rand.MakeSeed());
  }
}

template <size_t N>
void RandomWalk<N>::RHS(double x, const libra::Vector<N>& state, libra::Vector<N>& rhs) {
  UNUSED(x);  // TODO: consider the x is really need for this function

  for (size_t i = 0; i < N; ++i) {
    if (state[i] > limit_[i])
      rhs[i] = -fabs(nrs_[i]);
    else if (state[i] < -limit_[i])
      rhs[i] = fabs(nrs_[i]);
    else
      rhs[i] = nrs_[i];
  }
}

#endif  // S2E_LIBRARY_MATH_RANDOM_WALK_TFS_HPP_