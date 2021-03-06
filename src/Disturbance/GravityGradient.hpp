#ifndef __GravityGradient_H__
#define __GravityGradient_H__
#include <string>

#include "../Interface/LogOutput/ILoggable.h"
#include "../Library/math/MatVec.hpp"
#include "../Library/math/Matrix.hpp"
#include "../Library/math/Vector.hpp"
#include "SimpleDisturbance.h"

using libra::Matrix;
using libra::Vector;

class GravityGradient : public SimpleDisturbance {
 public:
  GravityGradient();  // mu is automatically set as earth's gravity
  GravityGradient(const double mu_m3_s2);
  virtual void Update(const LocalEnvironment& local_env, const Dynamics& dynamics);

  // r_b: Position vector of the earth [m] @ body frame, I_b: Inertia Tensor [kg*m^2]
  Vector<3> CalcTorque(const Vector<3> r_b, const Matrix<3, 3> I_b);

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  double mu_m3_s2_;
};

#endif  //__GravityGradient_H__
