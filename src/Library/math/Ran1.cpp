/*!
  \file   Ran1.cpp
  \author TAKISAWA <junichi@Hyperion>
  \date   Wed Sep 30 23:13:41 2009
  \brief  Ran1.hppの実装
*/
#include "Ran1.hpp"
using libra::Ran1;

Ran1::Ran1() : y_(0) { init_(); }

Ran1::Ran1(long seed) : ran0_(seed), y_(0) { init_(); }

void Ran1::init_() {
  // ran0_のウォームアップ
  for (int i = 0; i < 8; i++) {
    double temp = ran0_;
    static_cast<void>(temp);
  }
  // 切り混ぜ表を埋める。
  for (size_t i = 0; i < V_SIZE_; i++) {
    vec_[i] = ran0_;
  }
  //    for(size_t i=0; i<V_SIZE_; i++){ v_[i] = ran0_; }
}

Ran1::operator double() {
  double out = vec_[y_];
  vec_[y_] = ran0_;  // 次の乱数を補填
  y_ = (size_t)out * Ran0::M;
  y_ %= V_SIZE_;  // y <- [0 : V_SIZE_-1]

  return out;
}
