/**
 * @file quaternion.hpp
 * @brief Class for Quaternion
 */

#include "quaternion.hpp"

#include <cassert>
#include <cfloat>
#include <stdexcept>

namespace math {

Quaternion::Quaternion(const Vector<3>& rotation_axis, const double rotation_angle_rad) {
  double half_rotation_angle_rad = rotation_angle_rad * 0.5;
  quaternion_[3] = cos(half_rotation_angle_rad);

  // Vector<3> norm = Normalize(rotation_axis);
  // for(size_t i=0; i<3; ++i){ quaternion_[i] = norm[i]*sin(rotation_angle_rad); }
  for (size_t i = 0; i < 3; ++i) {
    quaternion_[i] = rotation_axis[i] * sin(half_rotation_angle_rad);
  }
}

Quaternion::Quaternion(const Vector<3>& vector_before, const Vector<3>& vector_after) {
  // Assert for zero vector
  assert(vector_before.CalcNorm() > DBL_EPSILON);
  assert(vector_after.CalcNorm() > DBL_EPSILON);
  // Normalize
  Vector<3> normalized_v_before = 1.0 / vector_before.CalcNorm() * vector_before;
  Vector<3> normalized_v_after = 1.0 / vector_after.CalcNorm() * vector_after;
  // inner product (=cosine of the CalcAngleTwoVectors_rad(theta) between two vectors)
  double ip = InnerProduct(normalized_v_before, normalized_v_after);
  // outer product (rotation rotation_axis for converting vector_before to vector_after)
  Vector<3> op = OuterProduct(normalized_v_before, normalized_v_after);

  if (ip > 1.0 - DBL_EPSILON) {  // if theta=0, then rotation is not need
    quaternion_[0] = 0.0;
    quaternion_[1] = 0.0;
    quaternion_[2] = 0.0;
    quaternion_[3] = 1.0;
  } else if (ip < -1.0 + DBL_EPSILON) {
    // if theta=180deg, the rotation rotation_axis can't be defined, so rotate vector_before manually
    Vector<3> rotation_axis = GenerateOrthogonalUnitVector(vector_before);
    quaternion_[0] = rotation_axis[0], quaternion_[1] = rotation_axis[1], quaternion_[2] = rotation_axis[2], quaternion_[3] = 0.0;
  } else {
    assert(op.CalcNorm() > 0.0);
    Vector<3> rotation_axis = 1.0 / op.CalcNorm() * op;
    double rotation_angle = acos(ip);
    quaternion_[0] = rotation_axis[0] * sin(0.5 * rotation_angle);
    quaternion_[1] = rotation_axis[1] * sin(0.5 * rotation_angle);
    quaternion_[2] = rotation_axis[2] * sin(0.5 * rotation_angle);
    quaternion_[3] = cos(0.5 * rotation_angle);
  }
}

Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs) {
  Quaternion temp;
  for (int i = 0; i < 4; ++i) {
    temp[i] = lhs[i] - rhs[i];
  }
  return temp;
}

Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs) {
  Quaternion temp;
  for (int i = 0; i < 4; ++i) {
    temp[i] = lhs[i] + rhs[i];
  }
  return temp;
}

Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs) {
  Quaternion temp;

  temp[0] = lhs[3] * rhs[0] - lhs[2] * rhs[1] + lhs[1] * rhs[2] + lhs[0] * rhs[3];
  temp[1] = lhs[2] * rhs[0] + lhs[3] * rhs[1] - lhs[0] * rhs[2] + lhs[1] * rhs[3];
  temp[2] = -lhs[1] * rhs[0] + lhs[0] * rhs[1] + lhs[3] * rhs[2] + lhs[2] * rhs[3];
  temp[3] = -lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] + lhs[3] * rhs[3];

  return temp;
}

Quaternion operator*(const Quaternion& lhs, const Vector<3>& rhs) {
  Quaternion temp;

  temp[0] = lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0];
  temp[1] = -lhs[0] * rhs[2] + lhs[2] * rhs[0] + lhs[3] * rhs[1];
  temp[2] = lhs[0] * rhs[1] - lhs[1] * rhs[0] + lhs[3] * rhs[2];
  temp[3] = -lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2];

  return temp;
}

Quaternion operator*(const double& lhs, const Quaternion& rhs) {
  Quaternion temp;
  for (size_t i = 0; i < 4; i++) {
    temp[i] = lhs * rhs[i];
  }
  return temp;
}

Quaternion Quaternion::Normalize(void) {
  double n = 0.0;
  for (int i = 0; i < 4; ++i) {
    n += pow(quaternion_[i], 2.0);
  }
  if (n == 0.0) {
    return quaternion_;
  }  // zero Quaternion

  n = 1.0 / sqrt(n);
  for (int i = 0; i < 4; ++i) {
    quaternion_[i] *= n;
  }

  return quaternion_;
}

Quaternion Quaternion::Conjugate(void) const {
  Quaternion temp(quaternion_);
  for (int i = 0; i < 3; ++i) {
    temp[i] *= -1.0;
  }
  return temp;
}

Matrix<3, 3> Quaternion::ConvertToDcm(void) const {
  Matrix<3, 3> dcm;

  dcm[0][0] = quaternion_[3] * quaternion_[3] + quaternion_[0] * quaternion_[0] - quaternion_[1] * quaternion_[1] - quaternion_[2] * quaternion_[2];
  dcm[0][1] = 2.0 * (quaternion_[0] * quaternion_[1] + quaternion_[3] * quaternion_[2]);
  dcm[0][2] = 2.0 * (quaternion_[0] * quaternion_[2] - quaternion_[3] * quaternion_[1]);

  dcm[1][0] = 2.0 * (quaternion_[0] * quaternion_[1] - quaternion_[3] * quaternion_[2]);
  dcm[1][1] = quaternion_[3] * quaternion_[3] - quaternion_[0] * quaternion_[0] + quaternion_[1] * quaternion_[1] - quaternion_[2] * quaternion_[2];
  dcm[1][2] = 2.0 * (quaternion_[1] * quaternion_[2] + quaternion_[3] * quaternion_[0]);

  dcm[2][0] = 2.0 * (quaternion_[0] * quaternion_[2] + quaternion_[3] * quaternion_[1]);
  dcm[2][1] = 2.0 * (quaternion_[1] * quaternion_[2] - quaternion_[3] * quaternion_[0]);
  dcm[2][2] = quaternion_[3] * quaternion_[3] - quaternion_[0] * quaternion_[0] - quaternion_[1] * quaternion_[1] + quaternion_[2] * quaternion_[2];

  return dcm;
}

Quaternion Quaternion::ConvertFromDcm(const Matrix<3, 3> dcm) {
  Quaternion q;
  q[0] = sqrt(1 + dcm[0][0] - dcm[1][1] - dcm[2][2]) / 2;
  q[1] = sqrt(1 - dcm[0][0] + dcm[1][1] - dcm[2][2]) / 2;
  q[2] = sqrt(1 - dcm[0][0] - dcm[1][1] + dcm[2][2]) / 2;
  q[3] = sqrt(1 + dcm[0][0] + dcm[1][1] + dcm[2][2]) / 2;
  double max_value = 0;
  int max_index = 0;
  for (int i = 0; i < 4; i++) {
    // Scan maximum value index
    if (std::abs(q[i]) > max_value) {
      max_value = std::abs(q[i]);
      max_index = i;
    }
  }
  // Use the maximum value as denominator
  switch (max_index) {
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

Vector<3> Quaternion::ConvertToEuler(void) const {
  auto dcm = this->ConvertToDcm();
  Vector<3> euler;
  euler[0] = atan2(dcm[1][2], dcm[2][2]);
  euler[1] = atan2(-dcm[0][2], sqrt(dcm[1][2] * dcm[1][2] + dcm[2][2] * dcm[2][2]));
  euler[2] = atan2(dcm[0][1], dcm[0][0]);
  return euler;
}

Quaternion Quaternion::ConvertFromEuler(const Vector<3> euler) {
  double esin[3], ecos[3];
  for (int i = 0; i < 3; i++) {
    esin[i] = sin(euler[i]);
    ecos[i] = cos(euler[i]);
  }
  Matrix<3, 3> dcm;
  dcm[0][0] = ecos[1] * ecos[2];
  dcm[0][1] = ecos[1] * esin[2];
  dcm[0][2] = -esin[1];
  dcm[1][0] = -ecos[0] * esin[2] + esin[0] * esin[1] * ecos[2];
  dcm[1][1] = ecos[0] * ecos[2] + esin[0] * esin[1] * esin[2];
  dcm[1][2] = esin[0] * ecos[1];
  dcm[2][0] = esin[0] * esin[2] + ecos[0] * esin[1] * ecos[2];
  dcm[2][1] = -esin[0] * ecos[2] + ecos[0] * esin[1] * esin[2];
  dcm[2][2] = ecos[0] * ecos[1];
  return Quaternion::ConvertFromDcm(dcm);
}

Vector<3> Quaternion::FrameConversion(const Vector<3>& vector) const {
  Quaternion conjugate = Conjugate();
  Quaternion temp1 = conjugate * vector;
  Quaternion temp2 = temp1 * quaternion_;
  Vector<3> answer;
  for (int i = 0; i < 3; ++i) {
    answer[i] = temp2[i];
  }
  return answer;
}

Vector<3> Quaternion::InverseFrameConversion(const Vector<3>& vector) const {
  Quaternion conjugate = Conjugate();
  Quaternion temp1 = quaternion_ * vector;
  Quaternion temp2 = temp1 * conjugate;
  Vector<3> answer;
  for (int i = 0; i < 3; ++i) {
    answer[i] = temp2[i];
  }
  return answer;
}

Vector<4> Quaternion::ConvertToVector() { return quaternion_; }

}  // namespace math
