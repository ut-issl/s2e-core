/**
 * @file Quaternion.hpp
 * @brief Class for Quaternion
 */

#ifndef QUATERNION_HPP_
#define QUATERNION_HPP_

#include "Matrix.hpp"
#include "Vector.hpp"

namespace libra {

/**
 * @class Quaternion
 * @brief Class for Quaternion
 */
class Quaternion {
 public:
  /**
   * @fn Quaternion
   * @brief Default constructor without any initialization
   */
  inline Quaternion();
  /**
   * @fn Quaternion
   * @brief Constructor with initialization
   * @param [in] q0: The first element of Quaternion (X)
   * @param [in] q1: The second element of Quaternion (Y)
   * @param [in] q2: The third element of Quaternion (Z)
   * @param [in] q3: The fourth element of Quaternion (W)
   */
  inline Quaternion(double q0, double q1, double q2, double q3);
  /**
   * @fn Quaternion
   * @brief Constructor initialized with vector
   * @param [in] cv: Vector storing quaternion
   */
  inline Quaternion(const Vector<4>& cv);
  /**
   * @fn Quaternion
   * @brief Constructor initialized with rotation expression
   * @param [in] axis: Rotation axis normalized vector
   * @param [in] rot: Rotation angle [rad]
   */
  Quaternion(const Vector<3>& axis, double rot);
  /**
   * @fn Quaternion
   * @brief Constructor initialized with rotates v_before to match v_after
   * @param [in] v_before: Vector before rotation
   * @param [in] v_after: Vector after rotation
   */
  Quaternion(const Vector<3>& v_before, const Vector<3>& v_after);

  /**
   * @fn Operator =
   * @brief Substitute Vector value to Quaternion
   * @param [in] cv: Vector
   * @return Quaternion
   */
  inline Quaternion& operator=(const Vector<4>& cv);

  /**
   * @fn Cast operator to const Vector<4>
   * @brief Return const reference to the internal Vector<4>
   * @note Users can use Quaternion as Vector<4> object
   * @return Const reference to the internal Vector<4>
   */
  inline operator const Vector<4>&() const;

  /**
   * @fn set
   * @brief Set the quaternion elements
   * @param [in] q0: The first element of Quaternion (X)
   * @param [in] q1: The second element of Quaternion (Y)
   * @param [in] q2: The third element of Quaternion (Z)
   * @param [in] q3: The fourth element of Quaternion (W)
   */
  void set(double q0 = 0.0, double q1 = 0.0, double q2 = 0.0, double q3 = 1.0);

  /**
   * @fn Cast operator
   * @brief Operator to directly access the element like array with [] operator
   */
  inline operator double*();

  /**
   * @fn Cast operator (const ver.)
   * @brief Operator to directly access the element like array with [] operator
   */
  inline operator const double*() const;

  /**
   * @fn normalize
   * @brief Normalize the quaternion
   * @return Normalized quaternion
   */
  Quaternion normalize(void);

  /**
   * @fn conjugate
   * @brief Calculate conjugate quaternion
   * @return Conjugated quaternion
   */
  Quaternion conjugate(void) const;

  /**
   * @fn toDCM
   * @brief Convert quaternion to Direction Cosine Matrix
   * @return DCM
   */
  Matrix<3, 3> toDCM(void) const;

  /**
   * @fn fromDCM
   * @brief Convert Direction Cosine Matrix to quaternion
   * @param [in] dcm: DCM
   * @return Quaternion
   */
  static Quaternion fromDCM(Matrix<3, 3> dcm);

  /**
   * @fn toEuler
   * @brief Convert quaternion to 3-2-1 Euler angles
   * @return Euler angle (1, 2, 3 order)
   */
  Vector<3> toEuler(void) const;

  /**
   * @fn fromEuler
   * @brief Convert Euler angle to quaternion
   * @param [in] euler: 3-2-1 Euler angle (1, 2, 3 order)
   * @return Quaternion
   */
  static Quaternion fromEuler(Vector<3> euler);

  /**
   * @fn frame_conv
   * @brief Frame conversion for the given target vector with the quaternion
   * @param [in] cv: Target vector
   * @return Converted vector
   */
  Vector<3> frame_conv(const Vector<3>& cv);

  /**
   * @fn frame_conv_inv
   * @brief Frame conversion for the given target vector with the inverse quaternion
   * @param [in] cv: Target vector
   * @return Converted vector
   */
  Vector<3> frame_conv_inv(const Vector<3>& cv);

  /**
   * @fn toVector
   * @brief Convert quaternion to Vector<4>
   * @return Vector<4>
   */
  Vector<4> toVector();

 private:
  Vector<4> q_;  //!< Vector to store the element of quaternion
};

/**
 * @fn operator+
 * @brief Add quaternions
 * @param [in] lhs: Left hand side quaternion
 * @param [in] rhs: Right hand side quaternion
 * @return Quaternion
 */
Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs);

/**
 * @fn operator-
 * @brief Subtract quaternions
 * @param [in] lhs: Left hand side quaternion
 * @param [in] rhs: Right hand side quaternion
 * @return Quaternion
 */
Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs);

/**
 * @fn operator*
 * @brief Multiply quaternions
 * @param [in] lhs: Left hand side quaternion
 * @param [in] rhs: Right hand side quaternion
 * @return Quaternion
 */
Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs);

/**
 * @fn operator*
 * @brief Multiply quaternion and vector
 * @param [in] lhs: Left hand side quaternion
 * @param [in] rhs: Right hand side vector
 * @return Quaternion
 */
Quaternion operator*(const Quaternion& lhs, const Vector<3>& rhs);

/**
 * @fn operator*
 * @brief Multiply scalar and quaternion
 * @param [in] lhs: Left hand side scalar
 * @param [in] rhs: Right hand side quaternion
 * @return Quaternion
 */
Quaternion operator*(const double& lhs, const Quaternion& rhs);
}  // namespace libra

#include "Quaternion_ifs.hpp"  // inline function definisions.

#endif  // QUATERNION_HPP_
