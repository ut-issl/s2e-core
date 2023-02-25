/**
 * @file matrix.hpp
 * @brief Matrix class to handle math matrix with template
 */

#ifndef S2E_LIBRARY_MATH_MATRIX_HPP_
#define S2E_LIBRARY_MATH_MATRIX_HPP_

#include <cstddef>   // for size_t
#include <iostream>  // for ostream, cout

namespace libra {

/**
 * @class Matrix
 * @brief Matrix class to handle math matrix with template
 */
template <size_t R, size_t C, typename T = double>
class Matrix {
 public:
  /**
   * @fn Matrix
   * @brief Default constructor without any initialization
   */
  inline Matrix() {}

  /**
   * @fn Matrix
   * @brief Constructor with initialize the elements as all same value
   * @param [in] n: The value for initializing
   */
  Matrix(const T& n);

  typedef T (*TP)[C];         //!< Define the pointer of the array as TP type
  typedef const T (*CTP)[C];  //!< Define the const pointer of the array as CTP type

  /**
   * @fn GetRowLength
   * @brief Return row number
   */
  inline size_t GetRowLength() const { return R; }

  /**
   * @fn GetColumnLength
   * @brief Return column number
   */
  inline size_t GetColumnLength() const { return C; }

  /**
   * @fn Cast operator to directly access the elements
   * @brief Operator to access the elements similar with the 2D-array using `[]`
   * @return Pointer to the data storing array
   */
  inline operator TP() { return matrix_; }

  /**
   * @fn Cast operator to directly access the elements (const ver.)
   * @brief Operator to access the elements similar with the 2D-array using `[]`
   * @return Const pointer to the data storing array
   */
  inline operator CTP() const { return matrix_; }

  /**
   * @fn Operator ()
   * @brief Operator to access the element value
   * @details This operator has assertion to detect range over
   * @param [in] row: Target row number
   * @param [in] column: Target column number
   * @return Value of the target element
   */
  inline T& operator()(size_t row, size_t column) {
    if (!ISValidRange(row, column)) {
      throw std::invalid_argument("Argument exceeds the range of matrix.");
    }
    return matrix_[row][column];
  }

  /**
   * @fn Operator ()
   * @brief Operator to access the element value (const ver.)
   * @details This operator has assertion to detect range over
   * @param [in] row: Target row number
   * @param [in] column: Target column number
   * @return Value of the target element
   */
  inline const T& operator()(size_t row, size_t column) const {
    if (!ISValidRange(row, column)) {
      throw std::invalid_argument("Argument exceeds the range of matrix.");
    }
    return matrix_[row][column];
  }

  /**
   * @fn Operator +=
   * @brief Operator to add Matrix
   * @note The element type should has += operator
   * @param [in] m: Adding matrix
   * @return Result of added matrix
   */
  const Matrix<R, C, T>& operator+=(const Matrix<R, C, T>& m);

  /**
   * @fn Operator -=
   * @brief Operator to subtract Matrix
   * @note The element type should has -= operator
   * @param [in] m: Subtracting matrix
   * @return Result of subtracted matrix
   */
  const Matrix<R, C, T>& operator-=(const Matrix<R, C, T>& m);

  /**
   * @fn Operator *=
   * @brief Operator to multiply scalar for all elements
   * @note The element type should has *= operator
   * @param [in] n: Multiplying scalar value
   * @return Result of multiplied matrix
   */
  const Matrix<R, C, T>& operator*=(const T& n);

  /**
   * @fn Operator /=
   * @brief Operator to divide scalar for all elements
   * @note The element type should has /= operator
   * @param [in] n: Dividing scalar value
   * @return Result of multiplied matrix
   */
  const Matrix<R, C, T>& operator/=(const T& n);

 private:
  T matrix_[R][C];  //!< Array to save the elements

  /**
   * @fn ISValidRange
   * @brief Judge the target row/column number is in the range
   * @param [in] row: Target row number
   * @param [in] column: Target column number
   * @return True: row/column number is in the range
   */
  inline bool ISValidRange(size_t row, size_t column) { return (row < R && column < C); }
};

/**
 * @fn fill_up
 * @brief Fill up all elements with same value
 * @param [in] m: Target matrix
 * @param [in] t: Scalar value to fill up
 */
template <size_t R, size_t C, typename T>
void FillUp(Matrix<R, C, T>& m, const T& t);

/**
 * @fn CalcTrace
 * @brief Calculate and return the CalcTrace of matrix
 * @param [in] m: Target matrix
 * @return Trace of the matrix
 */
template <size_t N, typename T>
T CalcTrace(const Matrix<N, N, T>& m);

/**
 * @fn print
 * @brief Generate all elements to outstream
 * @param [in] m: Target matrix
 * @param [in] delimiter: Delimiter (Default: tab)
 * @param [out] stream: Output target(Default: cout)
 */
template <size_t R, size_t C, typename T>
void Print(const Matrix<R, C, T>& m, char delimiter = '\t', std::ostream& stream = std::cout);

/**
 * @fn operator +
 * @brief Add two matrices
 * @param [in] lhs: Left hand side matrix
 * @param [in] rhs: Right hand side matrix
 * @return Result of added matrix
 */
template <size_t R, size_t C, typename T>
const Matrix<R, C, T> operator+(const Matrix<R, C, T>& lhs, const Matrix<R, C, T>& rhs);

/**
 * @fn operator -
 * @brief Subtract two matrices
 * @param [in] lhs: Left hand side matrix
 * @param [in] rhs: Right hand side matrix
 * @return Result of subtracted matrix
 */
template <size_t R, size_t C, typename T>
const Matrix<R, C, T> operator-(const Matrix<R, C, T>& lhs, const Matrix<R, C, T>& rhs);

/**
 * @fn operator *
 * @brief Multiply scar and matrix
 * @param [in] lhs: Left hand side scalar
 * @param [in] rhs: Right hand side matrix
 * @return Result of multiplied matrix
 */
template <size_t R, size_t C, typename T>
const Matrix<R, C, T> operator*(const T& lhs, const Matrix<R, C, T>& rhs);

/**
 * @fn operator *
 * @brief Multiply two matrices
 * @param [in] lhs: Left hand side matrix
 * @param [in] rhs: Right hand side matrix
 * @return Result of multiplied matrix
 */
template <size_t R, size_t C1, size_t C2, typename T>
const Matrix<R, C2, T> operator*(const Matrix<R, C1, T>& lhs, const Matrix<C1, C2, T>& rhs);

/**
 * @fn Transpose
 * @brief Calculate and return transposed matrix
 * @param [in] m: Target matrix
 * @return Result of transposed matrix
 */
template <size_t R, size_t C, typename T>
const Matrix<C, R, T> Transpose(const Matrix<R, C, T>& m);

/**
 * @fn Unitalize
 * @brief Rewrite the input matrix as the identity matrix
 * @note Warning: m is overwritten.
 * @param [in/out] m: Target matrix
 * @return The identity matrix
 */
template <size_t R, typename T>
Matrix<R, R, T>& Unitalize(Matrix<R, R, T>& m);

/**
 * @fn MakeIdentityMatrix
 * @brief Generate identity matrix
 * @return The identity matrix
 */
template <size_t R, typename T = double>
Matrix<R, R, T> MakeIdentityMatrix();

/**
 * @fn MakeRotationMatrixX
 * @brief Generate 3*3 rotation matrix around X-axis
 * @param [in] theta_rad: Rotation angle [rad]
 * @return Rotation matrix
 */
template <size_t R = 3, typename T = double>
Matrix<R, R, T> MakeRotationMatrixX(const double& theta_rad);

/**
 * @fn MakeRotationMatrixY
 * @brief Generate 3*3 rotation matrix around Y-axis
 * @param [in] theta_rad: Rotation angle [rad]
 * @return Rotation matrix
 */
template <size_t R = 3, typename T = double>
Matrix<R, R, T> MakeRotationMatrixY(const double& theta_rad);

/**
 * @fn MakeRotationMatrixZ
 * @brief Generate 3*3 rotation matrix around Z-axis
 * @param [in] theta_rad: Rotation angle [rad]
 * @return Rotation matrix
 */
template <size_t R = 3, typename T = double>
Matrix<R, R, T> MakeRotationMatrixZ(const double& theta_rad);

}  // namespace libra

#include "matrix_template_functions.hpp"

#endif  // S2E_LIBRARY_MATH_MATRIX_HPP_
