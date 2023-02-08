/**
 * @file matrix_vector.hpp
 * @brief Template library for Matrix-Vector calculation
 */

#ifndef S2E_LIBRARY_MATH_MATRIX_VECTOR_HPP_
#define S2E_LIBRARY_MATH_MATRIX_VECTOR_HPP_P

#include "vector.hpp"
#include "matrix.hpp"

namespace libra {

/**
 * @fn operator*
 * @brief Multiply matrix and vector
 * @param [in] m: Target matrix
 * @param [in] m: Target vector
 * @return Result of multiplied matrix
 */
template <size_t R, size_t C, typename TM, typename TC>
Vector<R, TC> operator*(const Matrix<R, C, TM>& m, const Vector<C, TC>& v);

/**
 * @fn invert
 * @brief Calculate inverse matrix
 * @param [in] a: Target matrix
 * @return Result of invert matrix
 */
template <std::size_t N>
Matrix<N, N> invert(const Matrix<N, N>& a);

/**
 * @fn ludcmp
 * @brief LU decomposition
 * @note Warning: a is overwritten.
 * @param [in/out] a: Target matrix
 * @param [in] index: Array to store row/column switch information
 * @return Result of LU decomposed matrix
 */
template <std::size_t N>
Matrix<N, N>& ludcmp(Matrix<N, N>& a, unsigned int index[]);

/**
 * @fn lubksb
 * @brief Solve linear system of equation with LU decomposition
 * @param [in] a: LU decomposed coefficient matrix
 * @param [in] index: Array to store row/column switch information
 * @param [in] b: Right hand side vector of the linear system
 * @return Result vector
 */
template <std::size_t N>
Vector<N>& lubksb(const Matrix<N, N>& a, const unsigned int index[], Vector<N>& b);

}  // namespace libra

#include "matrix_vector_impl.hpp"

#endif  // S2E_LIBRARY_MATH_MATRIX_VECTOR_HPP_
