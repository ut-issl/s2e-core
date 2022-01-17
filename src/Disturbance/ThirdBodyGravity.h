#pragma once
#include <string>

#include "../Interface/LogOutput/ILoggable.h"
#include "../Library/math/Vector.hpp"
#include "AccelerationDisturbance.h"
#include <cassert>
#include <set>

class ThirdBodyGravity : public AccelerationDisturbance {
public:
  ThirdBodyGravity(std::set<std::string> third_body_list);
  ~ThirdBodyGravity();
  virtual void Update(const LocalEnvironment &local_env,
                      const Dynamics &dynamics);

private:
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  libra::Vector<3> CalcAcceleration(libra::Vector<3> s, libra::Vector<3> sr,
                                    double GM);

  std::set<std::string> third_body_list_;
  libra::Vector<3> thirdbody_acc_i_{0};
};
