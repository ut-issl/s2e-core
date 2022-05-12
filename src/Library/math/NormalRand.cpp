/*!
  \file   NormalRand.cpp
  \author TAKISAWA Jun'ichi.
  \date   Sat Oct  3 02:53:44 2009
  \brief  NormalRand.hppの実装
*/
#include "NormalRand.hpp"
using libra::NormalRand;

#include <cfloat>  //DBL_EPSILON
#include <cmath>   //sqrt, log;

NormalRand::NormalRand() : avg_(0.0), stddev_(1.0), holder_(0.0), is_empty_(true) {}

NormalRand::NormalRand(double avg, double stddev) : avg_(avg), stddev_(stddev), holder_(0.0), is_empty_(true) {}

NormalRand::NormalRand(double avg, double stddev, long seed) throw() : avg_(avg), stddev_(stddev), rand_(seed), holder_(0.0), is_empty_(true) {}

NormalRand::operator double() {
  if (is_empty_) {
    double v1, v2, rsq;
    do {
      v1 = 2.0 * double(rand_) - 1.0;
      v2 = 2.0 * rand_ - 1.0;
      rsq = v1 * v1 + v2 * v2;
    } while (rsq >= 1.0 || rsq < DBL_EPSILON);
    double fac = std::sqrt(-2.0 * std::log(rsq) / rsq);

    holder_ = v1 * fac;
    is_empty_ = false;

    return v2 * fac * stddev_ + avg_;
  } else {
    is_empty_ = true;
    return holder_ * stddev_ + avg_;
  }
}
