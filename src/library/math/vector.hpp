/**
 * @file vector.hpp
 * @brief Class for mathematical vector
 */

#ifndef S2E_LIBRARY_MATH_VECTOR_HPP_
#define S2E_LIBRARY_MATH_VECTOR_HPP_

#include <cstddef>   // for size_t
#include <iostream>  // for ostream, cout

#define dot InnerProduct
#define cross OuterProduct

namespace libra {
/**
 * @class Vector
 * @brief Class for mathematical vector
 */
template <size_t N, typename T = double>
class Vector {
 public:
  /**
   * @fn Vector
   * @brief Constructor without any initialization
   */
  inline Vector() {}
  /**
   * @fn Vector
   * @brief Constructor with initialize the elements as all same value
   * @param [in] n: The value for initializing
   */
  explicit Vector(const T& n);

  /**
   * @fn dim
   * @brief Return number of elements
   */
  inline size_t GetLength() const { return N; }

  /**
   * @fn FillUp
   * @brief Fill up all elements with same value
   * @param [in] v: Target vector
   * @param [in] n: Scalar value to fill up
   */
  void FillUp(const T& n);

  /**
   * @fn Print
   * @brief Generate all elements to outstream
   * @param [in] v: Target vector
   * @param [in] delimiter: Delimiter (Default: tab)
   * @param [out] stream: Output target(Default: cout)
   */
  void Print(char delimiter = '\t', std::ostream& stream = std::cout);

  /**
   * @fn CalcNorm
   * @brief Calculate norm of vector
   * @param [in] v: Target vector
   * @return Norm of the vector
   */
  double CalcNorm() const;

  /**
   * @fn Cast operator to directly access the elements
   * @brief Operator to access the elements similar with the 1D-array using `[]`
   * @return Pointer to the data storing array
   */
  inline operator T*() { return vector_; }

  /**
   * @fn Cast operator to directly access the elements (const ver.)
   * @brief Operator to access the elements similar with the 1D-array using `[]`
   * @return Pointer to the data storing array
   */
  inline operator const T*() const { return vector_; }

  /**
   * @fn Operator ()
   * @brief Operator to access the element value
   * @details This operator has assertion to detect range over
   * @param [in] position: Target element number
   * @return Value of the target element
   */
  inline T& operator()(std::size_t position) {
    if (N <= position) {
      throw std::invalid_argument("Argument exceeds Vector's dimension.");
    }
    return vector_[position];
  }

  /**
   * @fn Operator ()
   * @brief Operator to access the element value (const ver.)
   * @details This operator has assertion to detect range over
   * @param [in] position: Target element number
   * @return Value of the target element
   */
  inline T operator()(std::size_t position) const {
    if (N <= position) {
      throw std::invalid_argument("Argument exceeds Vector's dimension.");
    }
    return vector_[position];
  }

  /**
   * @fn Operator +=
   * @brief Operator to add Vector
   * @note The element type should has += operator
   * @param [in] v: Adding vector
   * @return Result of added vector
   */
  Vector<N, T>& operator+=(const Vector<N, T>& v);

  /**
   * @fn Operator -=
   * @brief Operator to subtract Vector
   * @note The element type should has -= operator
   * @param [in] v: Subtracting vector
   * @return Result of subtracted vector
   */
  Vector<N, T>& operator-=(const Vector<N, T>& v);

  /**
   * @fn Operator *=
   * @brief Operator to multiply scalar for all elements
   * @note The element type should has *= operator
   * @param [in] n: Multiplying scalar value
   * @return Result of multiplied vector
   */
  Vector<N, T>& operator*=(const T& n);

  /**
   * @fn Operator /=
   * @brief Operator to divide scalar for all elements
   * @note The element type should has /= operator
   * @param [in] n: Dividing scalar value
   * @return Result of multiplied vector
   */
  Vector<N, T>& operator/=(const T& n);

  /**
   * @fn Operator -
   * @brief Return negative value of the vector
   * @return negative vector
   */
  Vector<N, T> operator-() const;

 private:
  T vector_[N];  //!< Array to store elements
};

/**
 * @fn operator +
 * @brief Add two vectors
 * @param [in] lhs: Left hand side vector
 * @param [in] rhs: Right hand side vector
 * @return Result of added vector
 */
template <size_t N, typename T>
const Vector<N, T> operator+(const Vector<N, T>& lhs, const Vector<N, T>& rhs);

/**
 * @fn operator -
 * @brief Subtract two vectors
 * @param [in] lhs: Left hand side vector
 * @param [in] rhs: Right hand side vector
 * @return Result of subtracted vector
 */
template <size_t N, typename T>
const Vector<N, T> operator-(const Vector<N, T>& lhs, const Vector<N, T>& rhs);

/**
 * @fn operator *
 * @brief Multiply scar and vector
 * @param [in] lhs: Left hand side scalar
 * @param [in] rhs: Right hand side vector
 * @return Result of multiplied vector
 */
template <size_t N, typename T>
const Vector<N, T> operator*(const T& lhs, const Vector<N, T>& rhs);

/**
 * @fn InnerProduct
 * @brief Inner product of two vectors
 * @param [in] lhs: Left hand side vector
 * @param [in] rhs: Right hand side vector
 * @return Result of scalar value
 */
template <size_t N, typename T>
const T InnerProduct(const Vector<N, T>& lhs, const Vector<N, T>& rhs);

/**
 * @fn OuterProduct
 * @brief Outer product of two vectors
 * @param [in] lhs: Left hand side vector
 * @param [in] rhs: Right hand side vector
 * @return Result vector
 */
template <typename T>
const Vector<3, T> OuterProduct(const Vector<3, T>& lhs, const Vector<3, T>& rhs);

/**
 * @fn Normalize
 * @brief Normalize the target vector
 * @note Warning: v is overwritten.
 * @param [in/out] v: Target vector
 * @return Normalized vector
 */
template <size_t N>
Vector<N, double>& Normalize(Vector<N, double>& v);

/**
 * @fn CalcAngleTwoVectors_rad
 * @brief Calculate angle between two vectors
 * @param [in] v1: First vector
 * @param [in] v2: Second vector
 * @return Angle between v1 and v2 [rad]
 */
template <size_t N>
double CalcAngleTwoVectors_rad(const Vector<N, double>& v1, const Vector<N, double>& v2);

/**
 * @fn ConvertFrameOrthogonal2Polar
 * @brief Convert orthogonal coordinate (x, y, z) to Polar coordinate (r, theta, phi)
 * @note 0 <= theta < pi and 0 <= phi < 2pi
 *       Return zero vector when input is zero vector. Return phi = 0 when input vector is on the Z-axis
 * @param [in] orthogonal: Vector in orthogonal coordinate
 * @return Vector in Polar coordinate
 */
Vector<3, double> ConvertFrameOrthogonal2Polar(const Vector<3, double>& orthogonal);

/**
 * @fn GenerateOrthogonalUnitVector
 * @brief Generate one unit vector orthogonal to the given 3D vector
 * @note Vectors orthogonal to the other vector have rotational degree of freedom, which are determined arbitrarily in this function.
 * @param [in] v: Given vector
 * @return Generated unit vector that is orthogonal to v
 */
Vector<3, double> GenerateOrthogonalUnitVector(const Vector<3, double>& v);

}  // namespace libra

#include "vector_template_functions.hpp"

#endif  // S2E_LIBRARY_MATH_VECTOR_HPP_
