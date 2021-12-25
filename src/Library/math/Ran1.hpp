/*!
  \file   Ran1.hpp
  \author TAKISAWA Jun'ichi.
  \date   Wed Sep 30 22:54:03 2009
  \brief  乱数生成ルーチン1
  "NUMERICAL RECIPES in C"のp.207-p.208に記載されている関数ran1のC++実装。
  Park and Miller の乗算合同法に切り混ぜを加えたもの。
*/
#ifndef RAN1_HPP_
#define RAN1_HPP_

#include "Ran0.hpp"

#include <cstddef> // size_t

namespace libra
{

class Ran1
{
public:
  //! コンストラクタ
  /*!
    デフォルトの種で乱数オブジェクトを生成する。
  */
  Ran1();

  //! コンストラクタ (種指定版)
  /*!
    乱数の種を引数として指定する。
    0は種に指定できず、指定した場合にはstd::invalid_argument例外が発生する。
    \param seed 乱数の種
  */
  explicit Ran1(long seed);

  //! double型へのキャスト演算子
  /*!
    オブジェクトがdouble型へキャストされるたびに、新たな乱数を生成する。
   */
  operator double();
private:
  //! 切り混ぜ表の初期化処理を行う関数
  void init_();

  Ran0 ran0_;
  //! 切り混ぜ表の要素数
  static const std::size_t V_SIZE_ = 32;
  //! 切り混ぜ表
  double v_[V_SIZE_];
  //! 切り混ぜ表の取得位置を格納するメンバ。
  std::size_t y_;
  double vec_[V_SIZE_];
};

} // libra

#endif //RAN1_HPP_
