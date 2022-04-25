#pragma once

#include <Library/utils/Macros.hpp>

template <size_t N>
RandomWalk<N>::RandomWalk(double step_width, const libra::Vector<N>& stddev, const libra::Vector<N>& limit)
    : libra::ODE<N>(step_width), limit_(limit) {
  // 標準偏差設定
  for (size_t i = 0; i < N; ++i) {
    nrs_[i].set_param(0.0, stddev[i]);  // g_rand.MakeSeed()
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