#ifndef __GEOPOTENTIAL_H__
#define __GEOPOTENTIAL_H__
#include <string>

#include "../Interface/LogOutput/ILoggable.h"
#include "../Library/math/MatVec.hpp"
#include "../Library/math/Matrix.hpp"
#include "../Library/math/Vector.hpp"
#include "AccelerationDisturbance.h"

using libra::Matrix;
using libra::Vector;

class GeoPotential : public AccelerationDisturbance {
public:
  GeoPotential(const int degree, const std::string file_path);
  virtual void Update(const LocalEnvironment &local_env,
                      const Dynamics &dynamics);
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  void CalcAccelerationECEF(const Vector<3> &position_ecef);
  bool ReadCoefficientsEGM96(std::string file_name);

private:
  int degree_;
  vector<vector<double>> c_;
  vector<vector<double>> s_;
  Vector<3> acc_ecef_;
  // calculation
  double r = 0.0, x = 0.0, y = 0.0, z = 0.0;
  int n = 0, m = 0;
  void v_w_nn_update(double *v_nn, double *w_nn, const double v_prev,
                     const double w_prev);
  void v_w_nm_update(double *v_nm, double *w_nm, const double v_prev,
                     const double w_prev, const double v_prev2,
                     const double w_prev2);
  // debug
  Vector<3> debug_pos_ecef_;
  double time_ = 0.0;
};

#endif //__GEOPOTENTIAL_H__
