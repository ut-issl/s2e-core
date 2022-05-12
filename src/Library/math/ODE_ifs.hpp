/*!
  \file   ODE_ifs.hpp
  \author TAKISAWA Jun'ichi.
  \date   Tue May 10 10:19:13 2011
  \brief  ODEクラスのinline関数定義
*/
#ifndef ODE_IFS_HPP_
#define ODE_IFS_HPP_

namespace libra {

template <size_t N>
ODE<N>::~ODE() {}

template <size_t N>
double ODE<N>::step_width() const {
  return step_width_;
}

template <size_t N>
double ODE<N>::x() const {
  return x_;
}

template <size_t N>
const Vector<N>& ODE<N>::state() const {
  return state_;
}

template <size_t N>
double ODE<N>::operator[](int n) const {
  return state_[n];
}

template <size_t N>
const Vector<N>& ODE<N>::rhs() const {
  return rhs_;
}

template <size_t N>
libra::Vector<N>& ODE<N>::state() {
  return state_;
}

}  // namespace libra

#endif  // ODE_IFS_HPP_
