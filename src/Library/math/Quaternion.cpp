/*!
  \file   Quaternion.cpp
  \author TAKISAWA, Jun'ichi.
  \date   Fri Jul 10 23:55:34 2009
  \brief  Quaternion.hppの実装
*/

#include "Quaternion.hpp"
#include <stdexcept>


namespace libra
{

Quaternion::Quaternion(const Vector<3>& axis,
                       double rot)
{
  rot *= 0.5; // 回転角の1/2を計算
  q_[3] = cos(rot);

  //Vector<3> norm = normalize(axis);
  //for(size_t i=0; i<3; ++i){ q_[i] = norm[i]*sin(rot); }
  for (size_t i = 0; i<3; ++i){
      q_[i] = axis[i] * sin(rot);
  }
}

Quaternion operator-(const Quaternion& lhs,
                     const Quaternion& rhs)
{
  Quaternion temp;
  for(int i=0; i<4; ++i){ temp[i] = lhs[i]-rhs[i]; }
  return temp;
}

Quaternion operator+(const Quaternion& lhs,
                     const Quaternion& rhs)
{
  Quaternion temp;
  for(int i=0; i<4; ++i){ temp[i] = lhs[i]+rhs[i]; }
  return temp;
}

Quaternion operator*(const Quaternion &lhs,
                     const Quaternion &rhs)
{
  Quaternion temp;

  temp[0] = lhs[3]*rhs[0]-lhs[2]*rhs[1]+lhs[1]*rhs[2]+lhs[0]*rhs[3];
  temp[1] = lhs[2]*rhs[0]+lhs[3]*rhs[1]-lhs[0]*rhs[2]+lhs[1]*rhs[3];
  temp[2] =-lhs[1]*rhs[0]+lhs[0]*rhs[1]+lhs[3]*rhs[2]+lhs[2]*rhs[3];
  temp[3] =-lhs[0]*rhs[0]-lhs[1]*rhs[1]-lhs[2]*rhs[2]+lhs[3]*rhs[3];

  return temp;
}

Quaternion operator*(const Quaternion &lhs,
                     const Vector<3> &rhs)
{
  Quaternion temp;

  temp[0] = lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0];
  temp[1] = -lhs[0] * rhs[2] + lhs[2] * rhs[0] + lhs[3] * rhs[1];
  temp[2] = lhs[0] * rhs[1] - lhs[1] * rhs[0] + lhs[3] * rhs[2];
  temp[3] = -lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2];

  return temp;
}

Quaternion Quaternion::normalize(void)
{
  double n = 0.0;
  for(int i=0; i<4; ++i){ n += pow(q_[i], 2.0); }
  if(n == 0.0){ return q_; } //零Quaternion

  n = 1.0/sqrt(n);
  for(int i=0; i<4; ++i){ q_[i]*=n; }

  return q_;
}

Quaternion Quaternion::conjugate(void) const
{
  Quaternion temp(q_);
  for(int i=0; i<3; ++i){ temp[i] *= -1.0; }
  return temp;
}

Matrix<3,3> Quaternion::toDCM(void) const
{
  Matrix<3, 3> dcm;

  dcm[0][0] = q_[3] * q_[3] + q_[0] * q_[0] - q_[1] * q_[1] - q_[2] * q_[2];
  dcm[0][1] = 2.0*(q_[0] * q_[1] + q_[3] * q_[2]);
  dcm[0][2] = 2.0*(q_[0] * q_[2] - q_[3] * q_[1]);

  dcm[1][0] = 2.0*(q_[0] * q_[1] - q_[3] * q_[2]);
  dcm[1][1] = q_[3] * q_[3] - q_[0] * q_[0] + q_[1] * q_[1] - q_[2] * q_[2];
  dcm[1][2] = 2.0*(q_[1] * q_[2] + q_[3] * q_[0]);

  dcm[2][0] = 2.0*(q_[0] * q_[2] + q_[3] * q_[1]);
  dcm[2][1] = 2.0*(q_[1] * q_[2] - q_[3] * q_[0]);
  dcm[2][2] = q_[3] * q_[3] - q_[0] * q_[0] - q_[1] * q_[1] + q_[2] * q_[2];

  return dcm;
}

Quaternion Quaternion::fromDCM(Matrix<3, 3> dcm)
{
  Quaternion q;
  q[0] = sqrt(1 + dcm[0][0] - dcm[1][1] - dcm[2][2]) / 2;
  q[1] = sqrt(1 - dcm[0][0] + dcm[1][1] - dcm[2][2]) / 2;
  q[2] = sqrt(1 - dcm[0][0] - dcm[1][1] + dcm[2][2]) / 2;
  q[3] = sqrt(1 + dcm[0][0] + dcm[1][1] + dcm[2][2]) / 2;
  double maxval = 0;
  int maxidx = 0;
  for (int i = 0; i < 4; i++)
  {
    // 最大値のインデックスをスキャン
    if (abs(q[i]) > maxval)
    {
      maxval = abs(q[i]);
      maxidx = i;
    }
  }
  // 最大値を分母にして変換
  switch (maxidx)
  {
  case 0:
    q[1] = (dcm[0][1] + dcm[1][0]) / (4 * q[0]);
    q[2] = (dcm[0][2] + dcm[2][0]) / (4 * q[0]);
    q[3] = (dcm[1][2] - dcm[2][1]) / (4 * q[0]);
    break;
  case 1:
    q[0] = (dcm[0][1] + dcm[1][0]) / (4 * q[1]);
    q[2] = (dcm[2][1] + dcm[1][2]) / (4 * q[1]);
    q[3] = (dcm[2][0] - dcm[0][2]) / (4 * q[1]);
    break;
  case 2:
    q[0] = (dcm[2][0] + dcm[0][2]) / (4 * q[2]);
    q[1] = (dcm[2][1] + dcm[1][2]) / (4 * q[2]);
    q[3] = (dcm[0][1] - dcm[1][0]) / (4 * q[2]);
    break;
  case 3:
    q[0] = (dcm[1][2] - dcm[2][1]) / (4 * q[3]);
    q[1] = (dcm[2][0] - dcm[0][2]) / (4 * q[3]);
    q[2] = (dcm[0][1] - dcm[1][0]) / (4 * q[3]);
    break;
  }
  return q;
}

Vector<3> Quaternion::toEuler(void) const
{
  auto dcm = this->toDCM();
  Vector<3> eul;
  eul[0] = atan2(dcm[1][2], dcm[2][2]);
  eul[1] = atan2(-dcm[0][2], sqrt(dcm[1][2] * dcm[1][2] + dcm[2][2] * dcm[2][2]));
  eul[2] = atan2(dcm[0][1], dcm[0][0]);
  return eul;
}

Quaternion Quaternion::fromEuler(Vector<3> euler)
{
  double esin[3], ecos[3];
  for (int i = 0; i < 3; i++)
  {
    esin[i] = sin(euler[i]);
    ecos[i] = cos(euler[i]);
  }
  Matrix<3, 3> dcm;
  dcm[0][0] =  ecos[1] * ecos[2];
  dcm[0][1] =  ecos[1] * esin[2];
  dcm[0][2] = -esin[1];
  dcm[1][0] = -ecos[0] * esin[2] + esin[0] * esin[1] * ecos[2];
  dcm[1][1] =  ecos[0] * ecos[2] + esin[0] * esin[1] * esin[2];
  dcm[1][2] =  esin[0] * ecos[1];
  dcm[2][0] =  esin[0] * esin[2] + ecos[0] * esin[1] * ecos[2];
  dcm[2][1] = -esin[0] * ecos[2] + ecos[0] * esin[1] * esin[2];
  dcm[2][2] =  ecos[0] * ecos[1];
  return Quaternion::fromDCM(dcm);
}

Vector<3> Quaternion::frame_conv(const Vector<3>& v)
{
  Quaternion conj = conjugate();
  Quaternion temp1 = conj*v;
  Quaternion temp2 = temp1*q_;
  Vector<3> ans;
  for (int i = 0; i<3; ++i){ ans[i] = temp2[i]; }
  return ans;
}

Vector<3> Quaternion::frame_conv_inv(const Vector<3>& cv)
{
  Quaternion conj = conjugate();
  Quaternion temp1 = q_ * cv;
  Quaternion temp2 = temp1 * conj;
  Vector<3> ans;
  for (int i = 0; i<3; ++i) { ans[i] = temp2[i]; }
  return ans;
}

Vector<4> Quaternion::toVector()
{
  return q_;
}

} //libra
