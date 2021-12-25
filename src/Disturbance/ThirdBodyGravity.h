#pragma once
#include"../Interface/LogOutput/ILoggable.h"
#include"AccelerationDisturbance.h"
#include<set>
#include<cassert>
#include "../Library/math/Vector.hpp"

class ThirdBodyGravity : public AccelerationDisturbance
{
public:
  ThirdBodyGravity(std::set<string> third_body_list);
  ~ThirdBodyGravity();
  virtual void Update(Envir& env, const Spacecraft & spacecraft);

private:
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

  libra::Vector<3> CalcAcceleration(libra::Vector<3> s, libra::Vector<3> sr, double GM);

  std::set<string> third_body_list_;
  libra::Vector<3> thirdbody_acc_i_{ 0 };
};

