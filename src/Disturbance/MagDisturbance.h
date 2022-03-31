#ifndef __MagDisturbance_H__
#define __MagDisturbance_H__

#include <string>

#include "../Library/math/Vector.hpp"
using libra::Vector;

#include "../Interface/LogOutput/ILoggable.h"
#include "../Simulation/Spacecraft/Structure/RMMParams.h"
#include "SimpleDisturbance.h"

class MagDisturbance : public SimpleDisturbance {
  Vector<3> rmm_b_;
  double mag_unit_;
  Vector<3> rmm_const_b_;
  double rmm_rwdev_;
  double rmm_rwlimit_;
  double rmm_wnvar_;

 public:
  MagDisturbance(const Vector<3>& rmm_const_b, const double rmm_rwdev, const double rmm_rwlimit, const double rmm_wnvar);
  void CalcRMM();
  Vector<3> CalcTorque(const Vector<3>& mag_b);
  virtual void Update(const LocalEnvironment& local_env, const Dynamics& dynamics);
  void PrintTorque();

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;
};

#endif  //__MagDisturbance_H__
