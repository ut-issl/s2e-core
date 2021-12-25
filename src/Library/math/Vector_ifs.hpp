/*!
  \file   Vector_ifs.hpp
  \author TAKISAWA, Jun'ichi.
  \date   Sun Apr 24 00:55:31 2011
  \brief  Vector.hppのinline関数実装
*/
#ifndef VECTOR_IFS_HPP_
#define VECTOR_IFS_HPP_

#include <stdexcept> // for invalid_argument.

namespace libra
{

template<size_t N, typename T>
Vector<N, T>::Vector(){}

template<size_t N, typename T>
size_t Vector<N, T>::dim() const { return N; }

template<size_t N, typename T>
Vector<N, T>::operator T*(){ return vector_; }

template<size_t N, typename T>
Vector<N, T>::operator const T*() const { return vector_; }

template<size_t N, typename T>
T& Vector<N, T>::operator()(std::size_t pos)
{
  if(N <= pos)
  {
    throw
      std::invalid_argument("Argument exceeds Vector's dimension.");
  }
  return vector_[pos];
}

template<size_t N, typename T>
T Vector<N, T>::operator()(std::size_t pos) const
{
  if(N <= pos)
  {
    throw
      std::invalid_argument("Argument exceeds Vector's deimension.");
  }
  return vector_[pos];
}

}

#endif // VECTOR_IFS_HPP_
