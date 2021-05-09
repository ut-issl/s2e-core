/*!
  \file   Quaternion.hpp
  \author TAKISAWA, Jun'ichi.
  \date   Fri Jul 10 23:36:06 2009
  \brief  Quaternionクラス

  Quaternionを取り扱う上で必要となる操作をまとめたクラス。
*/
#ifndef QUATERNION_HPP_
#define QUATERNION_HPP_

#include "Vector.hpp"
#include "Matrix.hpp"

namespace libra
{

class Quaternion
{
public:
  //! コンストラクタ
  /*!
    デフォルトコンストラクタ。一切の初期化処理を行わない。
   */
  inline Quaternion();

  //! コンストラクタ
  /*!
    指定された4値でQuaternionを生成する。
    \param q0 Quaternion第1成分
    \param q1 Quaternion第2成分
    \param q2 Quaternion第3成分
    \param q3 Quaternion第4成分
  */
  inline Quaternion(double q0, double q1, double q2, double q3);

  //! コンストラクタ
  /*!
    Vectorで指定された値でQuaternionをオブジェクトを生成する。
    \param cv Quaternion値を格納したVector
  */
  inline Quaternion(const Vector<4>& cv);

  //! コンストラクタ
  /*!
    回転角度と回転軸から座標変換を表すQuaternionを生成する。
    回転軸は内部で正規化処理を行うため単位ベクトルである必要はない。
    \param axis 回転軸
    \param rot 回転角度[rad]
  */
  Quaternion(const Vector<3>& axis,
             double rot);

  //! Constructor
  /*!
    Generate a Quaternion that rotates v_before to match v_after
    \param v_before Vector before rotation
    \param v_after Vector after rotation
  */
  Quaternion(const Vector<3>& v_before,
             const Vector<3>& v_after);

  //! Vectorからの代入演算子
  /*!
    Vectorで指定された値をQuaternionへ代入する。
    \param cv Quaternion値を格納したVector<4>
    \return 代入後の自身への参照
  */
  inline Quaternion& operator=(const Vector<4>& cv);

  //! const Vector<4>&へのキャスト演算子
  /*!
    Quaternion情報を保持するVector<4>へのconst参照を返す。
    QuaternionをVector<4>オブジェクトとして扱いたい場合に
    暗黙的な変換を行う。
    \return Quaternion内部のVector<4>へのconst参照
  */
  inline operator const Vector<4>&() const;

  //! Quaternion値設定メソッド
  /*!
    Quaternionの値を引数で指定された値に設定する。
    引数指定を省略した場合は(1.0, 0.0, 0.0, 0.0)^Tに設定される。
    \param q0 Quaternionの第1成分
    \param q1 Quaternionの第2成分
    \param q2 Quaternionの第3成分
    \param q3 Quaternionの第4成分
  */
  void set(double q0=0.0,
           double q1=0.0,
           double q2=0.0,
           double q3=1.0);

  //! 要素直接アクセス用キャスト演算子
  /*!
    Quaternionの各要素に配列と同様にアクセスする為のキャスト演算子。
    利用側は配列と同様[]演算子を用いて要素へのアクセスが可能となる。
  */
  inline operator double*();

  //! 要素直接アクセス用キャスト演算子 const版
  /*!
    constが指定されている場合でも各要素への参照を可能にする為の演算子定義。
    利用側は配列と同様[]演算子を用いて要素の参照が可能となる。
  */
  inline operator const double*() const;

  //! 正規化実行関数
  /*!
  Quaternionのノルムを1.0に正規化する。
  */
  Quaternion normalize(void);

  //! 共役Quaternion計算関数
  /*!
  与えられたQuaternionの共役Quaternionを返す
  \return 共役Quaternion
  */
  Quaternion conjugate(void) const;

  //! DCM生成関数
  /*!
  現在のQuaternionに対応するDCM(Discrete Cosine Matrix)を返す。
  \return Quaternionから計算されたDCM。
  */
  Matrix<3, 3> toDCM(void) const;

  //! DCM生成関数
  /*!
  与えられたDCMに対応するQuaternionを返す。
  \return DCMから計算されたQuaternion。
  */
  static Quaternion fromDCM(Matrix<3, 3> dcm);

  //! オイラー角生成関数
  /*!
  現在のQuaternionに対応する3-2-1のオイラー角を返す。
  \return Quaternionから計算されたオイラー角（1,2,3の順）
  */
  Vector<3> toEuler(void) const;

  //! オイラー角からQuaternionを生成する関数
  /*!
  与えられた3-2-1のオイラー角（1,2,3の順に与える）に対応するQuaternionを返す。
  \return オイラー角から計算されたQuaternion
  */
  static Quaternion fromEuler(Vector<3> euler);

  //! 座標変換計算関数
  /*!
  渡されたVectorをQuaternionで座標変換し、結果を返す。
  \param cv 変換対象Vector
  \return 変換結果のVector
  */
  Vector<3> frame_conv(const Vector<3>& cv);

  //! 座標変換計算関数
  /*!
  渡されたVectorをQuaternionの共役で座標変換し、結果を返す。
  \param cv 変換対象Vector
  \return 変換結果のVector
  */
  Vector<3> frame_conv_inv(const Vector<3>& cv);

  //! Quaternion to vector representation
  /*!
   \return Quaternion vector
   */
  Vector<4> toVector();

private:
  //! Quaternionの要素を格納する列ベクトル
  Vector<4> q_;
};

//! Quaternion同士の和演算子
Quaternion operator+(const Quaternion& lhs,
                     const Quaternion& rhs);

//! Quaternion同士の差演算子
Quaternion operator-(const Quaternion& lhs,
                     const Quaternion& rhs);

//! Quaternion同士の積演算子
Quaternion operator*(const Quaternion &lhs,
                     const Quaternion &rhs);

//! QuaternionとVectorの積演算子
Quaternion operator*(const Quaternion& lhs,
                     const Vector<3>& rhs);
} // libra

#include "Quaternion_ifs.hpp" // inline function definisions.

#endif //QUATERNION_HPP_
