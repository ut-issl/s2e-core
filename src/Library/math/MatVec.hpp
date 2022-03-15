/*!
  \file   MatVec.hpp
  \author TAKISAWA Jun'ichi.
  \date   Wed Oct 27 21:12:14 2010
  \brief  行列-ベクトル関連テンプレートライブラリ
*/
#ifndef MAT_VEC_HPP_
#define MAT_VEC_HPP_

#include "Matrix.hpp"
#include "Vector.hpp"

namespace libra {

//! MatrixとVectorの乗算演算子
/*!
  \param m 乗算対象Matrix
  \param v 乗算対象Vector
  \return 乗算結果
*/
template <size_t R, size_t C, typename TM, typename TC>
Vector<R, TC> operator*(const Matrix<R, C, TM>& m, const Vector<C, TC>& v);

//! 逆行列計算関数
/*!
  Matrixの逆行列を計算する。
  \param a 逆行列計算対象。
  \return 逆行列
*/
template <std::size_t N>
Matrix<N, N> invert(const Matrix<N, N>& a);

//! LU分解関数
/*!
  MatrixをLU分解する。
  \param a LU分解対象
  \param index 行入れ替え情報格納配列
  \return LU分解結果
*/
template <std::size_t N>
Matrix<N, N>& ludcmp(Matrix<N, N>& a, unsigned int index[]);

//! 一次連立方程式求解関数
/*!
  LU分解結果を利用して一次連立方程式の解を求める。
  \param a LU分解された一次連立方程式の係数Matrix
  \param index LU分解の行入れ替え情報
  \param b 一次連立方程式の右辺ベクトル
  \return 求解結果
*/
template <std::size_t N>
Vector<N>& lubksb(const Matrix<N, N>& a, const unsigned int index[],
                  Vector<N>& b);

}  // namespace libra

#include "MatVec_impl.hpp"

#endif  // MAT_VEC_HPP_
