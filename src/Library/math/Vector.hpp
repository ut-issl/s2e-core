/*!
  \file   Vector.hpp
  \author TAKISAWA Jun'ichi.
  \date   Sun Oct 24 13:49:13 2010
  \brief  数学のベクトルを扱うクラス
*/
#ifndef VECTOR_HPP_
#define VECTOR_HPP_

#include <cstddef> // for size_t
#include <iostream> // for ostream, cout

#define dot inner_product
#define cross outer_product

namespace libra
{
template<size_t N, typename T = double>
class Vector
{
public:
  //! コンストラクタ
  /*!
    ベクトル要素の初期化を一切行わないコンストラクタ。
  */
  inline Vector();

  //! コンストラクタ(一括初期化版)
  /*!
    ベクトル要素を指定した引数値で初期化する。
    ゼロクリアが必要な場合などに用いる。
    \param n 要素初期化値
  */
  explicit Vector(const T& n);

  //! 要素数取得関数
  /*!
    自身の要素数(次元数)を返す関数。
    \return 要素数
  */
  inline size_t dim() const;

  //! 要素直アクセス用のキャスト演算子
  /*!
    ベクトル各要素を配列と同様にアクセスする為のキャスト演算子。
    内部で保持しているデータ記録配列の先頭ポインタを返す。
    利用側は配列と同様[]演算子を利用して要素アクセスが可能になる。
    \return ベクトル先頭要素へのポインタ
  */
  inline operator T*();

  //! 要素直アクセス用のキャスト演算子 const版
  /*!
    constが指定されている場合であってもベクトル各要素の参照を
    可能にするための演算子定義。
    \return ベクトル先頭要素へのconstポインタ
  */
  inline operator const T*() const;

  //! 要素アクセス用()演算子
  /*!
    ベクトル各要素へのアクセスを提供する。キャスト演算子の場合と異なり、
    こちらは領域外指定に対する検知機構が実装されている。ベクトルの要素
    数を超えた範囲を指定した場合にはinvalid_argument例外が発生する。
    \param pos 参照したい要素番号
    \return 指定した位置の要素への参照
  */
  inline T& operator()(std::size_t pos);

  //! 要素アクセス用()演算子 const版
  /*!
    constが指定されている場合でも要素の参照を可能にする。
    非const版と同様領域外指定に対する検知機能が実装されている。
    \param pos 参照したい要素番号
    \return 指定した位置の要素
  */
  inline T operator()(std::size_t pos) const;

  //! 加算代入演算子
  /*!
    自身に指定されたVectorを加える。この動作にはVectorの要素の型
    について+=演算子が定義されている必要がある。
  */
  Vector<N, T>& operator+=(const Vector<N, T>& v);

  //! 減算代入演算子
  /*!
    自身から指定されたVectorを引く。この動作にはVectorの要素の型
    について-=演算子が定義されている必要がある。
  */
  Vector<N, T>& operator-=(const Vector<N, T>& v);

  //! 乗算代入演算子
  /*!
    自身の各要素に指定された値を乗じる。この動作にはVectorの要素の型
    について*=演算子が定義されている必要がある。
  */
  Vector<N, T>& operator*=(const T& n);

  //! 除算代入演算子
  /*!
    自身の各要素を指定された値で除する。この動作にはVectorの要素の型
    について/=演算子が定義されている必要がある。
  */
  Vector<N, T>& operator/=(const T& n);

  //! Vector減算演算子
  /*!
  Vectorの負反転を行う。
  */
  Vector<N, T> operator-() const;
private:
  //! 要素格納配列
  T vector_[N];
};

//! 全要素一括設定関数
/*!
  Vector全要素を指定された値に設定する。
  \param v 設定対象Vector
  \param n 設定値
*/
template<size_t N, typename T>
void fill_up(Vector<N, T>& v, const T& n);

//! 要素出力関数
/*!
  Vectorの全要素を指定された出力先(ostream)へ出力する。
  デフォルトの出力先はcoutである。
  各要素は引数で指定された区切り文字で区切って出力される。
  区切り文字のデフォルトはtabである。
  行末に改行などは出力されない。
  \param v 出力対象ベクトル
  \param delimiter 要素の区切り文字(デフォルトはtab)
  \param stream 出力先(デフォルトはcout)
*/
template<size_t N, typename T>
void print(const Vector<N, T>& v,
           char delimiter = '\t',
           std::ostream& stream = std::cout);

//! Vector加算演算子
/*!
  2つのVectorの加算を行う。
  \param lhs +演算子の左辺
  \param rhs +演算子の右辺
  \return 加算結果
*/
template<size_t N, typename T>
const Vector<N, T> operator+(const Vector<N, T>& lhs,
                             const Vector<N, T>& rhs);

//! Vector減算演算子
/*!
  2つのVectorの減算を行う。
  \param lhs -演算子の左辺
  \param rhs -演算子の右辺
  \return 減算結果
*/
template<size_t N, typename T>
const Vector<N, T> operator-(const Vector<N, T>& lhs,
                             const Vector<N, T>& rhs);

//! Vector係数倍演算子
/*!
  Vectorの各要素に指定された係数を乗じる
  \param lhs 係数
  \param rhs Vector
  \return 乗算結果
*/
template<size_t N, typename T>
const Vector<N, T> operator*(const T& lhs,
                             const Vector<N, T>& rhs);

//! 内積計算関数
/*!
  2つのVectorの内積を計算する
  \param lhs 左辺
  \param rhs 右辺
  \return 内積計算結果
*/
template<size_t N, typename T>
const T inner_product(const Vector<N, T>& lhs,
                      const Vector<N, T>& rhs);

//! 外積計算結果
/*!
  2つのVectorの外積を計算する
  \param lhs 左辺
  \param rhs 右辺
  \return 外積計算結果
*/
template<typename T>
const Vector<3, T> outer_product(const Vector<3, T>& lhs,
                                 const Vector<3, T>& rhs);

//! ノルム計算関数
/*!
  Vectorのノルムを計算する
  \param v 計算対象Vector
  \return ノルム計算結果
*/
template<size_t N>
double norm(const Vector<N, double>& v);

//! Vector正規化関数
/*!
  指定されたVectorをノルム1に正規化する。
  \param v 正規化対象Vector
  \return 正規化結果
*/
template<size_t N>
Vector<N, double>& normalize(Vector<N, double>& v);

//! ベクトル間角度計算関数
/*!
Vector同士の角度を計算する
\param v1,v2 計算対象Vector
\return 角度計算結果
*/
template<size_t N>
double angle(const Vector<N, double>& v1, const Vector<N, double>& v2);

//! 直交座標->球座標変換
/*!
  直交座標(Orthogonal Cordinates)表記のベクトルを球座標(Spherical
  Polar Cordinates)表記のベクトルへ変換する。
  ベクトル表記は(x, y, z) -> (r, theta, phi)である。thetaはベクトルとz
  軸のなす角、phiはベクトルのx-y平面への投影がx軸となす角度である。
  thetaの定義域は0<=theta<pi、phiの定義域は0<=phi<2piとする。
  零ベクトルが与えられた場合、球座標表記も零ベクトルとなる。与えられた
  ベクトルがz軸上の場合、phiの値は0.0とする。
  \param ortho 直交座標表記ベクトル(変換元)
  \return 球座標表記ベクトル(変換結果)
*/
Vector<3, double> ortho2spher(const Vector<3, double>& ortho);

//! 直交座標 -> 経度緯度座標変換
/*!
  直交座標(Orthogonal Cordinates)表記のベクトルを経度緯度(lonlat)表記の
  ベクトルへ変換する。
  ベクトル表記は(x, y, z) -> (h, lat, lon)である。
  \param ortho 直交座標表記ベクトル(変換元)
  \return 経度緯度座標表記ベクトル(変換結果)
*/
Vector<3, double> ortho2lonlat(const Vector<3, double>& ortho);

//! Generate one unit vector orthogonal to the given 3D vector
/*!
  NOTE: Vectors orthogonal to the other vector have rotational degree of freedom, which are determined arbitrarily in this function.
  \param v Given vector
  \return v_ortho Generated unit vector that is orthogonal to v
*/
Vector<3, double> GenerateOrthoUnitVector(const Vector<3, double>& v);

} //libra

#include "Vector_ifs.hpp" // inline function definisions.
#include "Vector_tfs.hpp" // template function definisions.

#endif //VECTOR_HPP_
