/*!
  \file   Quaternion_ifs.hpp
  \author TAKISAWA, Jun'ichi.
  \date   Wed May 18 22:59:03 2011
  \brief  Quaternionクラスのinline関数実装
*/
#ifndef QUATERNION_IFS_HPP_
#define QUATERNION_IFS_HPP_

namespace libra
{

Quaternion::Quaternion(){}

Quaternion::Quaternion(double q0, double q1, double q2, double q3)
{
  q_[0] = q0; q_[1] = q1; q_[2] = q2; q_[3] = q3;
}

Quaternion::Quaternion(const Vector<4>& q) : q_(q){}

Quaternion& Quaternion::operator=(const Vector<4>& q)
{
  q_ = q;
  return *this;
}

Quaternion::operator double*(){ return q_; }

Quaternion::operator const double*() const { return q_; }

Quaternion::operator const Vector<4>&() const { return q_; }

}

#endif // QUATERNION_IFS_HPP_
