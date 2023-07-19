/**
 * @file matrix_vector.hpp
 * @brief Template library for Matrix-Vector calculation
 */

#ifndef S2E_LIBRARY_MATH_MATRIX_VECTOR_HPP_
#define S2E_LIBRARY_MATH_MATRIX_VECTOR_HPP_

#include "matrix.hpp"
#include "vector.hpp"

namespace libra {

/**
 * @fn operator*
 * @brief Multiply matrix and vector
 * @param [in] matrix: Target matrix
 * @param [in] vector: Target vector
 * @return Result of multiplied matrix
 */
template <size_t R, size_t C, typename TM, typename TC>
Vector<R, TC> operator*(const Matrix<R, C, TM>& matrix, const Vector<C, TC>& vector);

/**
 * @fn CalcInverseMatrix
 * @brief Calculate inverse matrix
 * @param [in] matrix: Target matrix
 * @return Inverse matrix
 */
template <std::size_t N>
Matrix<N, N> CalcInverseMatrix(const Matrix<N, N>& matrix);

/**
 * @fn LuDecomposition
 * @brief LU decomposition
 * @note Warning: a is overwritten.
 * @param [in/out] matrix: Target matrix
 * @param [in] index: Array to store row/column switch information
 * @return Result of LU decomposed matrix
 */
template <std::size_t N>
Matrix<N, N>& LuDecomposition(Matrix<N, N>& matrix, size_t index[]);

/**
 * @fn SolveLinearSystemWithLu
 * @brief Solve linear system of equation with LU decomposition
 * @param [in] matrix: LU decomposed coefficient matrix
 * @param [in] index: Array to store row/column switch information
 * @param [in] vector: Right hand side vector of the linear system
 * @return Result vector
 */
template <std::size_t N>
Vector<N>& SolveLinearSystemWithLu(const Matrix<N, N>& matrix, const size_t index[], Vector<N>& vector);

}  // namespace libra

#include "matrix_vector_template_functions.hpp"

#endif  // S2E_LIBRARY_MATH_MATRIX_VECTOR_HPP_
