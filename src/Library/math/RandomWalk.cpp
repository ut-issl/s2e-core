/*!
  \file   MagRW.cpp
  \author KUDO, Takumi.
  \date   
  \brief  MagRW.hppの実装
*/
#include "RandomWalk.hpp"
using namespace libra;
#include "GlobalRand.h"

RandomWalk::RandomWalk(double step_width,
							const Vector<3>& stddev,
							const Vector<3>& limit)
  : ODE<3>(step_width), limit_(limit),
  nrs0_(0.0, stddev[0],g_rand.MakeSeed()), nrs1_(0.0, stddev[1],g_rand.MakeSeed()), nrs2_(0.0, stddev[2],g_rand.MakeSeed())
{
  // 標準偏差設定
  //for(size_t i=0; i<3; ++i){ nrs_[i].set_param(0.0, stddev[i]); }
}

void RandomWalk::RHS(double x,
						 const Vector<3>& state,
						 Vector<3>& rhs)
{
  rhs[0] = nrs0_;
  rhs[1] = nrs1_;
  rhs[2] = nrs2_;
  for(size_t i=0; i<3; ++i)
  {
    if(state[i] > limit_[i]){ rhs[i] = -fabs(rhs[i]); }
    else if(state[i] < -limit_[i]){ rhs[i] = fabs(rhs[i]); }
  }
}