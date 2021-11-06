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

  Atmosphere(string model, string fname, double gauss_stddev, 
             bool is_manual_param, double manual_f107, double manual_f107a, double manual_ap);
  double CalcAirDensity(double decyear, double endsec, Vector<3> lat_lon_alt);
  double GetAirDensity() const;
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:
  string model_; // 大気密度モデル
  string fname_; // iniファイルのパス
  double air_density_; // 大気密度 kg/m^3
  double gauss_stddev_; // 空気密度の標準偏差(密度に対する割合で定義)
  vector<nrlmsise_table> table_; // Space weather table
  bool is_table_imported_; // tableが読み込まれたかどうか
  bool is_manual_param_used_; // ユーザー指定の固定値（f10.7，ap値）を使うかどうか f10.7値については https://www.swpc.noaa.gov/phenomena/f107-cm-radio-emissions 参照
  double manual_daily_f107_; // ユーザー指定のf10.7値（1日あたりの値）
  double manual_average_f107_; // ユーザー指定のf10.7値（3ヶ月程度の平均値）
  double manual_ap_; // ユーザー指定のap値 ap値については http://wdc.kugi.kyoto-u.ac.jp/kp/kpexp-j.html 参照
//  double rw_stepwidth_;
//  double rw_stddev_;
//  double rw_limit_;
  double CalcStandard(double altitude_m);
  int GetSpaceWeatherTable(double decyear, double endsec);
  double AddNoise(double rho);
};

#endif //__Atmosphere.h__
