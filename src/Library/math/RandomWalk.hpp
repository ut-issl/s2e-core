/*!
  \file   MagRandomWalk.hpp
  \author Takumi Kudo
  \date   
  \brief  MagSensor用のランダムウォーク生成クラス
  \cite  TAKISAWA氏のST5RW.hppを改造．
*/
#ifndef RandomWalk_H_
#define RandomWalk_H_

#include "./ODE.hpp"
#include "./Vector.hpp"
#include "./NormalRand.hpp"

class RandomWalk : public libra::ODE<3>
{
public:
  //! コンストラクタ
  /*!
    \param step_width シミュレーションステップ幅
    \param stddev ランダムウォーク励起ノイズ標準偏差
    \param limit ランダムウォーク制限値
  */
  RandomWalk(double step_width,
				const libra::Vector<3>& stddev,
				const libra::Vector<3>& limit);

  virtual void RHS(double x,
                   const libra::Vector<3>& state,
                   libra::Vector<3>& rhs);
private:
  //! ランダムウォーク制限値
  libra::Vector<3> limit_;
  //! ランダムウォーク励起ノイズ源
  libra::NormalRand nrs0_,nrs1_,nrs2_;
};

#endif // ST5RW_HPP_
