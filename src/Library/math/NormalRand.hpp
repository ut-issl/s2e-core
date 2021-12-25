/*!
  \file   NormalRand.hpp
  \author TAKISAWA Jun'ichi.
  \date   Sat Oct  3 02:44:23 2009
  \brief  正規分布に従う乱数を生成する乱数クラス
  "NUMERICAL RECIPES in C" p.216-p.217に掲載されている、
  Box-Muller法を用いた正規ガウス分布生成アルゴリズムのC++実装。
*/
#ifndef NORMAL_RAND_HPP_
#define NORMAL_RAND_HPP_

#include "Ran1.hpp"
using libra::Ran1;

namespace libra
{

class NormalRand
{
public:
  //! コンストラクタ
  /*!
    平均0.0, 標準偏差1.0の標準正規乱数を出力するオブジェクトを生成する。
    乱数の種にはデフォルト値が利用される。
  */
  NormalRand();

  //! コンストラクタ (平均・標準偏差指定版)
  /*!
    指定された平均と標準偏差で正規乱数を出力するオブジェクトを生成する。
    乱数の種にはデフォルト値が利用される。
    \param avg 正規乱数の平均値
    \param stddev 正規乱数の標準偏差
  */
  NormalRand(double avg, double stddev);

  //! コンストラクタ
  /*!
    引数として指定された平均値、標準偏差の正規乱数を生成するオブジェクトを生成する。
    最後の引数は乱数の種である。
    全ての引数を省略した場合、オブジェクトは標準正規乱数を発生するよう初期化される。
    \param avg 正規乱数の平均値
    \param stddev 正規乱数の標準偏差
    \param seed 乱数の種
  */
  NormalRand(double avg, double stddev, long seed) throw();

  //! double型へのキャスト演算子
  /*!
    double型へのキャストが発生するたび、Box-Muller法を用いて正規乱数
    を生成する。
    \return 生成した正規乱数値
  */
  operator double();

  //! 平均値Getter
  inline double avg() const;

  //! 平均値Setter
  inline void avg(double avg);

  //! 標準偏差Getter
  inline double stddev() const;

  //! 標準偏差Setter
  inline void stddev(double stddev);

  //! 平均・標準偏差設定関数
  /*!
    正規乱数の平均値と標準偏差を設定する。
    \param avg 正規乱数の平均値
    \param stddev 正規乱数の標準偏差
  */
  inline void set_param(double avg, double stddev);
private:
  //! 平均値を保持するメンバ
  double avg_;

  //! 標準偏差を保持するメンバ
  double stddev_;

  //! Box-Muller法で利用する乱数源
  Ran1 rand_;

  //! 第二乱数保持メンバ
  /*!
    Box-Muller法では1度で2つの正規乱数が生成される。
    このメンバはその1つを次回呼び出しまで保持する。
    次回の呼び出しではこの値が正規乱数として返却されるので、Box-Muller法は2回に
    1回の頻度で実行される。
  */
  double holder_;

  //! メンバholder_が有効な値を持っているかを示すフラグ。
  bool is_empty_;
};

} // libra

#include "NormalRand_ifs.hpp" // inline function definisions.

#endif // NORMAL_RAND_HPP_
