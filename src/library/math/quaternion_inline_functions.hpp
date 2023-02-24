/**
 * @file quaternion_inline_functions.hpp
 * @brief Class for Quaternion (Inline functions)
 */

#ifndef S2E_LIBRARY_MATH_QUATERNION_INLINE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_QUATERNION_INLINE_FUNCTIONS_HPP_

namespace libra {

Quaternion::Quaternion() {}

Quaternion::Quaternion(double q_x, double q_y, double q_z, double q_w) {
  q_[0] = q_x;
  q_[1] = q_y;
  q_[2] = q_z;
  q_[3] = q_w;
}

Quaternion::Quaternion(const Vector<4>& quaternion_vector) : q_(quaternion_vector) {}

Quaternion& Quaternion::operator=(const Vector<4>& quaternion_vector) {
  q_ = quaternion_vector;
  return *this;
}

Quaternion::operator double*() { return q_; }

Quaternion::operator const double*() const { return q_; }

Quaternion::operator const Vector<4>&() const { return q_; }

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_QUATERNION_INLINE_FUNCTIONS_HPP_
