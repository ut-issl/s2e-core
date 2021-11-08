#pragma once

#include "../../Library/math/Vector.hpp"
#include "../../Library/math/Quaternion.hpp"
using namespace libra;

#include "../../Interface/LogOutput/ILoggable.h"

class EMDS : public ILoggable
{
public:
  bool IsCalcEnabled = true;

  EMDS(Vector<3> mm, Vector<3> displacement);
  ~EMDS();
  void Update(EMDS& other);
  inline Vector<3> GetTorque_b() { return torque_b_; }
  inline Vector<3> GetForce_b() { return force_b_; }

  virtual std::string GetLogHeader() const; 
  virtual std::string GetLogValue() const;

  // position: position of C.G. of s/c in inertia frame
  // quaternion: quaternion from inertia to body frame
  // current: electric current in ampere
  void SetParameters(Vector<3> position, Quaternion quaternion, double current);

private:
  void calc(Vector<3> d1_i, Vector<3> d2_i, Quaternion q1_ib, Quaternion q2_ib,
    Vector<3> r1_b, Vector<3> r2_b, double i1, double i2, Vector<3>* results);
  void calc_approx(Vector<3> d1_i, Vector<3> d2_i, Quaternion q1_ib, Quaternion q2_ib,
    Vector<3> r1_b, Vector<3> r2_b, double i1, double i2, double m_c0, Vector<3>* results);

  Vector<3> torque_b_ = Vector<3>(0);
  Vector<3> force_b_ = Vector<3>(0);
  Vector<3> mm_;

  // displacement
  Vector<3> dis_;
  
  // position(inertia frame)
  Vector<3> pos_;

  // quaternion(inertia to body)
  Quaternion q_;

  // electric current
  double i_ampere;
};
