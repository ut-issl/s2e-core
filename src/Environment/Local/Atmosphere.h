#pragma once

#ifndef __Atmosphere_H__
#define __Atmosphere_H__

#include "../../Library/math/Vector.hpp"
#include "../../Library/math/Quaternion.hpp"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../../Library/nrlmsise00/Wrapper_nrlmsise00.h"

using libra::Vector;
using libra::Quaternion;

class Atmosphere : public ILoggable
{
public:
  bool IsCalcEnabled = true;

  Atmosphere(string model, string fname, double gauss_stddev);
  double CalcAirDensity(double decyear, double endsec, Vector<3> lat_lon_alt);
  double GetAirDensity() const;
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:
  string model_;
  string fname_;
  double air_density_;
  double gauss_stddev_; //空気密度の標準偏差(密度に対する割合で定義)
  vector<nrlmsise_table> table_;
  bool is_table_imported_;
//  double rw_stepwidth_;
//  double rw_stddev_;
//  double rw_limit_;
  double CalcStandard(double altitude_m);
  int GetSpaceWeatherTable(double decyear, double endsec);
  double AddNoise(double rho);
};

#endif //__Atmosphere.h__
