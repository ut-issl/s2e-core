#pragma once

#include "./NormalRand.hpp"
#include "./ODE.hpp"
#include "./Vector.hpp"

template <size_t N>
class RandomWalk : public libra::ODE<N> {
 public:
  //! コンストラクタ
  /*!
    \param step_width シミュレーションステップ幅
    \param stddev ランダムウォーク励起ノイズ標準偏差
    \param limit ランダムウォーク制限値
  */
  RandomWalk(double step_width, const libra::Vector<N>& stddev, const libra::Vector<N>& limit);

  virtual void RHS(double x, const libra::Vector<N>& state, libra::Vector<N>& rhs);

 private:
  //! ランダムウォーク制限値
  libra::Vector<N> limit_;
  //! ランダムウォーク励起ノイズ源
  libra::NormalRand nrs_[N];
};

#include "./RandomWalk_tfs.hpp"  // template function definisions.