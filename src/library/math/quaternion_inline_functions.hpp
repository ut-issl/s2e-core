/**
 * @file quaternion_inline_functions.hpp
 * @brief Class for Quaternion (Inline functions)
 */

#ifndef S2E_LIBRARY_MATH_QUATERNION_INLINE_FUNCTIONS_HPP_
#define S2E_LIBRARY_MATH_QUATERNION_INLINE_FUNCTIONS_HPP_

namespace libra {

Quaternion::Quaternion() {}

Quaternion::Quaternion(const double quaternion_x, const double quaternion_y, const double quaternion_z, const double quaternion_w) {
  quaternion_[0] = quaternion_x;
  quaternion_[1] = quaternion_y;
  quaternion_[2] = quaternion_z;
  quaternion_[3] = quaternion_w;
}

Quaternion::Quaternion(const Vector<4>& quaternion_vector) : quaternion_(quaternion_vector) {}

Quaternion& Quaternion::operator=(const Vector<4>& quaternion_vector) {
  quaternion_ = quaternion_vector;
  return *this;
}

Quaternion::operator double*() { return quaternion_; }

Quaternion::operator const double*() const { return quaternion_; }

Quaternion::operator const Vector<4>&() const { return quaternion_; }

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_QUATERNION_INLINE_FUNCTIONS_HPP_
