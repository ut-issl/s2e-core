/*!
  \file   Matrix.hpp
  \author TAKISAWA, Jun'ichi.
  \date   Sat Oct 23 00:07:48 2010
  \brief  テンプレート行列クラス
  テンプレートを用いた行列クラス。数学の行列を取り扱う。
*/
#ifndef MATRIX_HPP_
#define MATRIX_HPP_

#include <cstddef> // for size_t
#include <iostream> // for ostream, cout

namespace libra
{

template<size_t R, size_t C, typename T = double>
class Matrix
{
public:
  //! コンストラクタ
  /*!
    要素初期化を一切行わないコンストラクタ。
  */
  inline Matrix();

  //! コンストラクタ(一括初期化版)
  /*!
    全要素を引数で指定された値で初期化する。
    ゼロクリアなどが必要な場合に用いる。
  */
  Matrix(const T& n);

  //! 行列格納配列へのポインタをTP型として定義。
  typedef T(*TP)[C];
  //! 行列格納配列へのconstポインタをCTP型として定義。
  typedef const T(*CTP)[C];

  //! 行数取得関数
  /*!
    行列の行数を返す。
    \return 行数
  */
  inline size_t row() const;

  //! 列数取得関数
  /*!
    行列の列数を返す。
    \return 列数
  */
  inline size_t column() const;

  //! 要素直接アセス用のキャスト演算子
  /*!
    行列各要素を2次元配列と同様にアクセスする為のキャスト演算子。
    内部で保持しているデータ記録配列へのポインタを返す。
    利用側は2次元配列同様[]演算子を利用して要素へのアクセスが可能になる。
    \return 行列データ記録配列へのポインタ
  */
  inline operator TP();

  //! 要素直接アクセス用のキャスト演算子 const版
  /*!
    constが指定されている場合でも行列各要素の参照を可能にする。
    \return 行列データ記録配列へのconstポインタ
  */
  inline operator CTP() const;

  //! 要素アクセス用()演算子
  /*!
    行列各要素へのアクセスを提供する。
    キャスト演算子の場合と異なり、この関数では領域外指定に対する検知機構が
    実装されている。行列の範囲外を指定した場合には"invalid_argumnet"例外
    が発生する。
    \param row 行列の行指定
    \param column 行列の列指定
    \return 指定された位置の要素
  */
  inline T& operator()(size_t row, size_t column);

  //! 要素アクセス用()演算子 const版
  /*!
    constが指定されている場合でも、要素の参照を可能にする。
    非const版と同様領域外指定に対する検知機構が実装されている。
    \param row 行列の行指定
    \param column 行列の列指定
    \return 指定された位置の要素
  */
  inline const T& operator()(size_t row, size_t column) const;

  //! 加算代入演算子
  /*!
    自身に引数で指定されたMatrixを加える。
    この操作を行うには行列の要素型について+=演算子が定義されている必要がある。
  */
  const Matrix<R, C, T>& operator+=(const Matrix<R, C, T>& m);

  //! 減算代入演算子
  /*!
    自身から引数で指定されたMatrixを減じる。
    この操作を行うには行列の要素型について-=演算子が定義されている必要がある。
  */
  const Matrix<R, C, T>& operator-=(const Matrix<R, C, T>& m);

  //! 乗算代入演算子
  /*!
    自身の各要素に引数で指定された係数を乗じる。
    この操作を行うには行列の要素型について*=演算子が定義されている必要がある。
  */
  const Matrix<R, C, T>& operator*=(const T& n);

  //! 除算代入演算子
  /*!
    自身の各要素を引数で指定された係数で除する。
    この操作を行うには行列の要素型について/=演算子が定義されている必要がある。
  */
  const Matrix<R, C, T>& operator/=(const T& n);

private:
  //! 行列要素記録用配列
  T matrix_[R][C];

  //! 行列領域確認関数
  /*!
    引数で指定された要素が行列の範囲内かどうかを判定する。
    \param row 行列の列指定
    \param column 行列の行指定
    \return 範囲内であればtrue それ以外はfalse
   */
  inline bool is_valid_range_(size_t row, size_t column);
};

//! 全要素一括設定関数
/*!
  Matrixの全要素を指定された値に設定する。
  \param m 設定対象Matrix
  \param t 設定値
*/
template<size_t R, size_t C, typename T>
void fill_up(Matrix<R, C, T>& m, const T& t);

//! 固有和計算関数
/*!
  Matrixの固有和を計算する。
  \param m 計算対象Matrix
  \return 固有和計算結果
*/
template<size_t N, typename T>
T trace(const Matrix<N, N, T>& m);

//! 要素出力関数
/*!
  Matrixの全要素を指定された出力先(ostream)へ出力する。
  デフォルトの出力先はcoutである。
  各要素は引数で指定された区切り文字区切りで出力され、各行ごとに改行が行われる。
  区切り文字のデフォルトはtabである。
  \param m 出力対象Matrix
  \param delimiter 要素の区切り文字(デフォルトはtab)
  \param stream 出力先(デフォルトはcout)
*/
template<size_t R, size_t C, typename T>
void print(const Matrix<R, C, T>& m,
           char delimiter = '\t',
           std::ostream& stream = std::cout);

//! Matrix加算演算子
/*!
  2つのMatrixの加算を行う。
  \param lhs +演算子の左辺
  \param rhs +演算子の右辺
  \return 加算結果
*/
template<size_t R, size_t C, typename T>
const Matrix<R, C, T> operator+(const Matrix<R, C, T>& lhs,
                                const Matrix<R, C, T>& rhs);

//! Matrix減算演算子
/*!
  2つのMatrixの減算を行う
  \param lhs -演算子の左辺
  \param rhs -演算子の右辺
  \return 減算結果
*/
template<size_t R, size_t C, typename T>
const Matrix<R, C, T> operator-(const Matrix<R, C, T>& lhs,
                                const Matrix<R, C, T>& rhs);

//! Matrix係数倍演算子
/*!
  Matrixの各要素に指定された係数を乗じる。
  \param lhs 係数
  \param rhs Matrix
  \return 係数乗算結果
*/
template<size_t R, size_t C, typename T>
const Matrix<R, C, T> operator*(const T& lhs,
                                const Matrix<R, C, T>& rhs);

//! Matrix乗算演算子
/*!
  2つのMatrixの乗算を行う
  \param lhs *演算子の左辺
  \param rhs *演算子の右辺
  \return 乗算結果
*/
template<size_t R, size_t C1, size_t C2, typename T>
const Matrix<R, C2, T> operator*(const Matrix<R, C1, T>& lhs,
                                 const Matrix<C1, C2, T>& rhs);

//! 転置行列計算関数
/*!
  指定された行列の転置行列を計算する。
  \param m 転置対象
  \return 転置結果
*/
template<size_t R, size_t C, typename T>
const Matrix<C, R, T> transpose(const Matrix<R, C, T>& m);

//! 単行列生成関数
/*!
  指定された正方行列を単位行列に設定する。
  引数で指定されたMatrixを直接単位行列へ書き換え、結果を返り値の形でも返す。
  \param m 単位行列設定対象
  \return 設定結果
*/
template<size_t R, typename T>
Matrix<R, R, T>& unitalize(Matrix<R, R, T>& m);

//! 単行列生成関数
/*!
指定された大きさの単位行列を生成する。
\return 生成結果
*/
template<size_t R, typename T = double>
Matrix<R, R, T> eye();

} //libra

#include "Matrix_ifs.hpp" // inline function definisions.
#include "Matrix_tfs.hpp" // template function definisions.

#endif //MATRIX_HPP_
