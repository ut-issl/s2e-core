#pragma once
#include <Library/math/Vector.hpp>
using libra::Vector;

class RMMParams
{
public:
  RMMParams(Vector<3> rmm_const_b, double rmm_rwdev, double rmm_rwlimit, double rmm_wnvar);
  ~RMMParams() {};
  inline const Vector<3>& GetRMMConst_b(void) const { return rmm_const_b_; }
  inline const double& GetRMMRWDev(void) const { return rmm_rwdev_; }
  inline const double& GetRMMRWLimit(void) const { return rmm_rwlimit_; }
  inline const double& GetRMMWNVar(void) const { return rmm_wnvar_; }

private:
  Vector<3> rmm_const_b_;
  double rmm_rwdev_;
  double rmm_rwlimit_;
  double rmm_wnvar_;
};

