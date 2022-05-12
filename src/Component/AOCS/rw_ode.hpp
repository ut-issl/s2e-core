/*
 * @file rw_ode.hpp
 * @brief リアクションホイールの指定された角速度までの推移の微分方程式(一時遅れ)
 * @author Shun Arahata
 * @date
 * @details ODEクラスを継承
 */
#ifndef __RW_ODE_H__
#define __RW_ODE_H__
#include <Library/math/ODE.hpp>
#include <Library/math/Vector.hpp>
#include <vector>

/**
 * @brief RWのトルクを求める微分方程式
 * @details ODEの実装
 */
class RwOde : public libra::ODE<1> {
 public:
  /**
   * @brief コンストラクタ
   * @param[in] step_width  計算ステップ幅
   * @param[in] first_order_lag　一次遅れ用定数
   * @param[in] init_angular_velocity 初期角速度
   * @param[in] target_angular_velocity 目標角速度
   */
  RwOde(double step_width, double init_angular_velocity, double target_angular_velocity, libra::Vector<3> lag_coef);
  /**
   * @brief 微分方程式の実装
   * @param[in] x　時間
   * @param[out] rhs 右辺
   * @param[out] state
   * @return void
   * @detail ode.hppにて純粋仮想関数として宣言
   */
  void RHS(double x, const libra::Vector<1>& state, libra::Vector<1>& rhs) override;

  /**
   *@brief 角速度の取得
   *@return 角速度
   *@detail 計算をさせているわけではないことに注意
   */
  double getAngularVelocity(void) const;

  /**
   *@brief 目標角速度の取得
   *@return void
   */

  void setTargetAngularVelocity(double angular_velocity);

  void setFirstOrderLag(double first_order_lag);

  void setSecondOrderCoef(double second_order_coef);

  void setLagCoef(libra::Vector<3> lag_coef);

 private:
  RwOde(double step_width);            //!基底クラスのコンストラクタ呼び出しの禁止
  libra::Vector<3> lag_coef_;          //!<一次遅れ用定数
  const double kInitAngularVelocity_;  //!初期角速度
  double target_angular_velocity_;     //!目標角速度
};

#endif  //__RW_ODE_H__
