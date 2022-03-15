/*!
  \file   ODE.hpp
  \author TAKISAWA Jun'ichi.
  \date   Sat Mar  7 10:14:04 2009
  \brief  常微分方程式を扱うクラス。
*/
#ifndef ODE_HPP_
#define ODE_HPP_

#include "./Vector.hpp"

namespace libra {

template <size_t N>
class ODE {
 public:
  //! コンストラクタ
  /*!
    第1引数にステップ幅をとる。
    \param step_width 計算ステップ幅
  */

  ODE(double step_width);

  //! デストラクタ
  inline virtual ~ODE();

  //! 微分方程式
  /*!
    解くべき微分方程式を継承側が実装する
    \param x 独立変数
    \param state 状態量
    \return 状態量微分値
  */
  virtual void RHS(double x, const Vector<N>& state, Vector<N>& rhs) = 0;

  //! 状態初期化メソッド
  /*!
    微分方程式の状態初期化を行う
    \param init_x 初期独立変数値
    \param init_cond 初期条件
  */
  void setup(double init_x, const Vector<N>& init_cond);

  void setStepWidth(double new_step);

  //! 刻み幅取得メソッド
  /*!
    独立変数の刻み幅を返す。
  */
  inline double step_width() const;

  //! 独立変数取得メソッド
  /*!
    現在の独立変数値を返す。
    独立変数はシミュレーションの経過とともに変化するものであり、
    独自に設定を行う関数は設けない仕様とする。
    初期条件の指定にはsetup()関数を用いること。
    \return 現在の独立変数値。
  */
  inline double x() const;

  //! 状態量ベクトル取得メソッド
  /*!
    状態量ベクトルへのconst参照を返す。
    状態量はシミュレーションの過程に従って変化するものであり、
    独自に設定を行う関数は設けない仕様とする。
    初期条件の指定にはsetup()関数を用いること。
    \return 状態量ベクトルへのconst参照
  */
  inline const Vector<N>& state() const;

  //! 状態量ベクトルの要素値を取得する
  /*!
    状態量ベクトルの要素値を返す。
    \param n 参照したい状態量の位置
    \return 引数に対応する位置の状態量
  */
  inline double operator[](int n) const;

  //! 状態量微分値取得メソッド
  /*!
    状態量微分値のベクトルへのconst参照を返す。
    このベクトルはシミュレーションの結果として生成される情報
    であり、外部からの直接設定は行えない仕様とする。
    ただし継承先のクラスについてはprotectedなメソッドで変更を
    許可する。
    \return 状態量微分値ベクトルへのconst参照
  */
  inline const Vector<N>& rhs() const;

  //! 状態更新メソッド
  ODE& operator++();

  //! 状態更新メソッド
  void Update();

 protected:
  inline libra::Vector<N>& state();

 private:
  //! 最新の独立変数値
  double x_;
  //! 最新の状態量
  Vector<N> state_;
  //! 最新の状態量微分値(微分方程式の右辺値)
  Vector<N> rhs_;
  //! 計算刻み幅
  double step_width_;
};

}  // namespace libra

#include "./ODE_ifs.hpp"  // inline function definisions.
#include "./ODE_tfs.hpp"  // template function definisions.

#endif  // ODE_HPP_
