#ifndef __MagEnvironment_H__
#define __MagEnvironment_H__

#include "../../Library/math/Vector.hpp"
using libra::Vector;
#include "../../Library/math/Quaternion.hpp"
using libra::Quaternion;

#include "../../Interface/LogOutput/ILoggable.h"

class MagEnvironment : public ILoggable {
public:
  bool IsCalcEnabled = true;

  MagEnvironment(std::string fname, double mag_rwdev, double mag_rwlimit,
                 double mag_wnvar);
  void CalcMag(double decyear, double side, Vector<3> lat_lon_alt,
               Quaternion q_i2b);
  Vector<3> GetMag_i() const;
  Vector<3> GetMag_b() const;

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

private:
  Vector<3> Mag_i_;
  Vector<3> Mag_b_;
  double mag_rwdev_;
  double mag_rwlimit_;
  double mag_wnvar_;
  std::string fname_;

  void AddNoise(double *mag_i_array);
};

#endif //__MagEnvironment_H__
