#pragma once

#include "../Library/math/Vector.hpp"
using libra::Vector;

class Disturbance {
public:
  Disturbance() {
    force_b_ = Vector<3>(0);
    torque_b_ = Vector<3>(0);
    acceleration_b_ = Vector<3>(0);
    acceleration_b_ = Vector<3>(0);
  }
  virtual inline Vector<3> GetTorque() { return torque_b_; }
  virtual inline Vector<3> GetForce() { return force_b_; }
  virtual inline Vector<3> GetAccelerationB() { return acceleration_b_; }
  virtual inline Vector<3> GetAccelerationI() { return acceleration_i_; }

  bool IsCalcEnabled = true;

protected:
  Vector<3> force_b_;
  Vector<3> torque_b_;
  Vector<3> acceleration_b_;
  Vector<3> acceleration_i_;
};
