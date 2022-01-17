#ifndef __GGDist_H__
#define __GGDist_H__
#include <string>

#include "../Interface/LogOutput/ILoggable.h"
#include "../Library/math/MatVec.hpp"
#include "../Library/math/Matrix.hpp"
#include "../Library/math/Vector.hpp"
#include "SimpleDisturbance.h"

using libra::Matrix;
using libra::Vector;

class GGDist : public SimpleDisturbance {
public:
  GGDist();
  GGDist(const double mu_e_input);
  virtual void Update(const LocalEnvironment &local_env,
                      const Dynamics &dynamics);
  Vector<3> CalcTorque(double R0, Vector<3> u_b, Matrix<3, 3> I_b);
  Vector<3> CalcTorque(Vector<3> r_b, Matrix<3, 3> I_b);
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

private:
  double mu_e_;
  double kilo_;
  Vector<3> ggtorque_b_;
};

#endif //__GGDist_H__
