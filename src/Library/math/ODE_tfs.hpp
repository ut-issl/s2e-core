/*!
  \file   ODE_tfs.hpp
  \author TAKISAWA Jun'ichi.
  \date   Sat Mar  7 10:19:50 2009
  \brief  ODE.hppの実装
*/
#ifndef ODE_TFS_HPP_
#define ODE_TFS_HPP_

namespace libra {

template <size_t N>
ODE<N>::ODE(double step_width) : x_(0.0), state_(0.0), rhs_(0.0), step_width_(step_width) {}

template <size_t N>
void ODE<N>::setup(double init_x, const Vector<N>& init_cond) {
  x_ = init_x;
  state_ = init_cond;
}

template <size_t N>
ODE<N>& ODE<N>::operator++() {
  Update();
  return *this;
}

template <size_t N>
void ODE<N>::Update() {
  RHS(x_, state_, rhs_);  // Current derivative calculation

  // 4次のRunge-Kutta係数計算
  Vector<N> k1(rhs_);
  k1 *= step_width_;
  Vector<N> k2(state_.dim());
  RHS(x_ + 0.5 * step_width_, state_ + 0.5 * k1, k2);
  k2 *= step_width_;
  Vector<N> k3(state_.dim());
  RHS(x_ + 0.5 * step_width_, state_ + 0.5 * k2, k3);
  k3 *= step_width_;
  Vector<N> k4(state_.dim());
  RHS(x_ + step_width_, state_ + k3, k4);
  k4 *= step_width_;

  state_ += (1.0 / 6.0) * (k1 + 2.0 * (k2 + k3) + k4);  // 状態量更新
  x_ += step_width_;                                    // 時刻更新
}

template <size_t N>
void ODE<N>::setStepWidth(double new_step) {
  step_width_ = new_step;
}
}  // namespace libra

#endif  // ODE_TFS_HPP_
