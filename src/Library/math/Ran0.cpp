/*!
  \file   Ran0.cpp
  \author TAKISAWA Jun'ichi.
  \date   Tue Sep 29 20:34:49 2009
  \brief  Ran0.hppの実装
*/
#include "Ran0.hpp"
using libra::Ran0;

#include <stdexcept>

const double Ran0::AM_ = 1.0/Ran0::M;

Ran0::Ran0() : seed_(0xdeadbeef){}

Ran0::Ran0(long seed) : seed_(seed)
{
  if(seed == 0){ throw std::invalid_argument("Ran0:: seed is 0."); }
}
void Ran0::init(long seed){
  if(seed == 0){ throw std::invalid_argument("Ran0:: seed is 0."); }
  seed_ = seed;
}

Ran0::operator double()
{
  long k = seed_/Q_;
  seed_ = A*(seed_-k*Q_)-R_*k;
  if(seed_ < 0){ seed_ += M; }
  return AM_*seed_;
}
