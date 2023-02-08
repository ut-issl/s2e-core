/**
 * @file vector.hpp
 * @brief Class for mathematical vector
 */

#ifndef S2E_LIBRARY_MATH_VECTOR_HPP_
#define S2E_LIBRARY_MATH_VECTOR_HPP_

#include <cstddef>   // for size_t
#include <iostream>  // for ostream, cout

#define dot inner_product
#define cross outer_product

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
  inline Vector();
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
  inline size_t dim() const;

  /**
   * @fn Cast operator to directly access the elements
   * @brief Operator to access the elements similar with the 1D-array using `[]`
   * @return Pointer to the data storing array
   */
  inline operator T*();

  /**
   * @fn Cast operator to directly access the elements (const ver.)
   * @brief Operator to access the elements similar with the 1D-array using `[]`
   * @return Pointer to the data storing array
   */
  inline operator const T*() const;

  /**
   * @fn Operator ()
   * @brief Operator to access the element value
   * @details This operator has assertion to detect range over
   * @param [in] pos: Target element number
   * @return Value of the target element
   */
  inline T& operator()(std::size_t pos);

  /**
   * @fn Operator ()
   * @brief Operator to access the element value (const ver.)
   * @details This operator has assertion to detect range over
   * @param [in] pos: Target element number
   * @return Value of the target element
   */
  inline T operator()(std::size_t pos) const;

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
 * @fn fill_up
 * @brief Fill up all elements with same value
 * @param [in] v: Target vector
 * @param [in] n: Scalar value to fill up
 */
template <size_t N, typename T>
void fill_up(Vector<N, T>& v, const T& n);

/**
 * @fn print
 * @brief Generate all elements to outstream
 * @param [in] v: Target vector
 * @param [in] delimiter: Delimiter (Default: tab)
 * @param [out] stream: Output target(Default: cout)
 */
template <size_t N, typename T>
void print(const Vector<N, T>& v, char delimiter = '\t', std::ostream& stream = std::cout);

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
 * @fn inner_product
 * @brief Inner product of two vectors
 * @param [in] lhs: Left hand side vector
 * @param [in] rhs: Right hand side vector
 * @return Result of scalar value
 */
template <size_t N, typename T>
const T inner_product(const Vector<N, T>& lhs, const Vector<N, T>& rhs);

/**
 * @fn outer_product
 * @brief Outer product of two vectors
 * @param [in] lhs: Left hand side vector
 * @param [in] rhs: Right hand side vector
 * @return Result vector
 */
template <typename T>
const Vector<3, T> outer_product(const Vector<3, T>& lhs, const Vector<3, T>& rhs);

/**
 * @fn norm
 * @brief Calculate norm of vector
 * @param [in] v: Target vector
 * @return Norm of the vector
 */
template <size_t N>
double norm(const Vector<N, double>& v);

/**
 * @fn normalize
 * @brief Normalize the target vector
 * @note Warning: v is overwritten.
 * @param [in/out] v: Target vector
 * @return Normalized vector
 */
template <size_t N>
Vector<N, double>& normalize(Vector<N, double>& v);

/**
 * @fn angle
 * @brief Calculate angle between two vectors
 * @param [in] v1: First vector
 * @param [in] v2: Second vector
 * @return Angle between v1 and v2 [rad]
 */
template <size_t N>
double angle(const Vector<N, double>& v1, const Vector<N, double>& v2);

/**
 * @fn ortho2spher
 * @brief Convert orthogonal coordinate (x, y, z) to Polar coordinate (r, theta, phi)
 * @note 0 <= theta < pi and 0 <= phi < 2pi
 *       Return zero vector when input is zero vector. Return phi = 0 when input vector is on the Z-axis
 * @param [in] ortho: Vector in orthogonal coordinate
 * @return Vector in Polar coordinate
 */
Vector<3, double> ortho2spher(const Vector<3, double>& ortho);

/**
 * @fn ortho2lonlat
 * @brief Convert orthogonal coordinate (x, y, z) to Geodetic coordinate (altitude, latitude, longitude)
 * @note TODO: Consider merge with GeodeticPosition class
 * @param [in] ortho: Vector in orthogonal coordinate
 * @return Vector in Geodetic coordinate
 */
Vector<3, double> ortho2lonlat(const Vector<3, double>& ortho);

/**
 * @fn GenerateOrthoUnitVector
 * @brief Generate one unit vector orthogonal to the given 3D vector
 * @note Vectors orthogonal to the other vector have rotational degree of freedom, which are determined arbitrarily in this function.
 * @param [in] v: Given vector
 * @return Generated unit vector that is orthogonal to v
 */
Vector<3, double> GenerateOrthoUnitVector(const Vector<3, double>& v);

}  // namespace libra

#include "vector_ifs.hpp"  // inline function definisions.
#include "vector_tfs.hpp"  // template function definisions.

#endif  // S2E_LIBRARY_MATH_VECTOR_HPP_
