/*!
  \file   Ran0.hpp
  \author TAKISAWA Jun'ichi.
  \date   Tue Sep 29 20:27:54 2009
  \brief  乱数生成ルーチン0
  "NUMERICAL RECIPES in C"のp.206に記載されている関数ran0のC++実装。
  Park and Millerの乗算合同法。
*/
#ifndef RAN0_HPP_
#define RAN0_HPP_

namespace libra {

class Ran0 {
 public:
  //! 乗算の係数a。
  static const long A = 16807;
  //! mod m。
  static const long M = 2147483647;

  //! コンストラクタ
  /*!
    デフォルトのシードで乱数クラスを生成する。
   */
  Ran0();

  //! コンストラクタ
  /*!
    引数で乱数列のseedを指定する。
    指定を省略した場合はデフォルト値0xdeadbeefが利用される。
  */
  explicit Ran0(long seed);

  //! 初期化
  /*
        指定した引数にseedを設定しなおす
  */
  void init(long seed);
  //! double型へのキャスト演算子
  /*!
    double型へのキャストが実行されるたびに新たな乱数を生成する。
    \return 生成した乱数値
  */
  operator double();

 private:
  static const double AM_;
  static const long Q_ = 127773;
  static const long R_ = 2836;

  long seed_;
};

}  // namespace libra
#endif  // RAN0_HPP_
