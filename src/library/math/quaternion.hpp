/**
 * @file quaternion.hpp
 * @brief Class for Quaternion
 */

#ifndef S2E_LIBRARY_MATH_QUATERNION_HPP_
#define S2E_LIBRARY_MATH_QUATERNION_HPP_

#include "matrix.hpp"
#include "vector.hpp"

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
   * @param [in] q_x: The first element of Quaternion (X)
   * @param [in] q_y: The second element of Quaternion (Y)
   * @param [in] q_z: The third element of Quaternion (Z)
   * @param [in] q_w: The fourth element of Quaternion (W)
   */
  inline Quaternion(double q_x, double q_y, double q_z, double q_w);
  /**
   * @fn Quaternion
   * @brief Constructor initialized with vector
   * @param [in] quaternion_vector: Vector storing quaternion
   */
  inline Quaternion(const Vector<4>& quaternion_vector);
  /**
   * @fn Quaternion
   * @brief Constructor initialized with rotation expression
   * @param [in] rotation_axis: Rotation axis normalized vector
   * @param [in] rotation_angle_rad: Rotation angle [rad]
   */
  Quaternion(const Vector<3>& rotation_axis, double rotation_angle_rad);
  /**
   * @fn Quaternion
   * @brief Constructor initialized with rotates vector_before to match vector_after
   * @param [in] vector_before: Vector before rotation
   * @param [in] vector_after: Vector after rotation
   */
  Quaternion(const Vector<3>& vector_before, const Vector<3>& vector_after);

  /**
   * @fn Operator =
   * @brief Substitute Vector value to Quaternion
   * @param [in] quaternion_vector: Vector
   * @return Quaternion
   */
  inline Quaternion& operator=(const Vector<4>& quaternion_vector);

  /**
   * @fn Cast operator to const Vector<4>
   * @brief Return const reference to the internal Vector<4>
   * @note Users can use Quaternion as Vector<4> object
   * @return Const reference to the internal Vector<4>
   */
  inline operator const Vector<4>&() const;

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
   * @fn Normalize
   * @brief Normalize the quaternion
   * @return Normalized quaternion
   */
  Quaternion Normalize(void);

  /**
   * @fn Conjugate
   * @brief Calculate Conjugate quaternion
   * @return Conjugated quaternion
   */
  Quaternion Conjugate(void) const;

  /**
   * @fn ConvertToDcm
   * @brief Convert quaternion to Direction Cosine Matrix
   * @return DCM
   */
  Matrix<3, 3> ConvertToDcm(void) const;

  /**
   * @fn ConvertFromDcm
   * @brief Convert Direction Cosine Matrix to quaternion
   * @param [in] dcm: DCM
   * @return Quaternion
   */
  static Quaternion ConvertFromDcm(Matrix<3, 3> dcm);

  /**
   * @fn ConvertToEuler
   * @brief Convert quaternion to 3-2-1 Euler angles
   * @return Euler angle (1, 2, 3 order)
   */
  Vector<3> ConvertToEuler(void) const;

  /**
   * @fn ConvertFromEuler
   * @brief Convert Euler angle to quaternion
   * @param [in] euler: 3-2-1 Euler angle (1, 2, 3 order)
   * @return Quaternion
   */
  static Quaternion ConvertFromEuler(Vector<3> euler);

  /**
   * @fn FrameConversion
   * @brief Frame conversion for the given target vector with the quaternion
   * @param [in] vector: Target vector
   * @return Converted vector
   */
  Vector<3> FrameConversion(const Vector<3>& vector) const;

  /**
   * @fn InverseFrameConversion
   * @brief Frame conversion for the given target vector with the inverse quaternion
   * @param [in] vector: Target vector
   * @return Converted vector
   */
  Vector<3> InverseFrameConversion(const Vector<3>& vector) const;

  /**
   * @fn ConvertToVector
   * @brief Convert quaternion to Vector<4>
   * @return Vector<4>
   */
  Vector<4> ConvertToVector();

 private:
  Vector<4> quaternion_;  //!< Vector to store the element of quaternion
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

#include "quaternion_inline_functions.hpp"

#endif  // S2E_LIBRARY_MATH_QUATERNION_HPP_
