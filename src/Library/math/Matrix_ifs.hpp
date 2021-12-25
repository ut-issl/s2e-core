/*!
  \file   Matrix_ifs.hpp
  \author TAKISAWA, Jun'ichi.
  \date   Sun Apr 24 14:21:50 2011
  \brief  Matrix.hppのinline関数実装
*/
#ifndef MATRIX_IFS_HPP_
#define MATRIX_IFS_HPP_

#include <stdexcept> // for invalid_argument

namespace libra
{

template<size_t R, size_t C, typename T>
Matrix<R, C, T>::Matrix(){};

template<size_t R, size_t C, typename T>
size_t Matrix<R, C, T>::row() const { return R; }

template<size_t R, size_t C, typename T>
size_t Matrix<R, C, T>::column() const { return C; }

template<size_t R, size_t C, typename T>
Matrix<R, C, T>::operator TP(){ return matrix_; }

template<size_t R, size_t C, typename T>
Matrix<R, C, T>::operator CTP() const{ return matrix_; }

template<size_t R, size_t C, typename T>
T& Matrix<R, C, T>::operator()(size_t row, size_t column)
{
  if(!is_valid_range_(row, column))
  {
    throw std::invalid_argument
      ("Argument exceeds the range of matrix.");
  }
  return matrix_[row][column];
}

template<size_t R, size_t C, typename T>
const T& Matrix<R, C, T>::operator()(size_t row, size_t column) const
{
  if(!is_valid_range_(row, column))
  {
    throw std::invalid_argument
      ("Argument exceeds the range of matrix.");
  }
  return matrix_[row][column];
}

template<size_t R, size_t C, typename T>
bool Matrix<R, C, T>::is_valid_range_(size_t row, size_t column)
{
  return (row<R && column<C);
}

} //libra

#endif // MATRIX_IFS_HPP_

