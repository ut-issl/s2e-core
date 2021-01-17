#include "Atmosphere.h"
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/NormalRand.hpp"
#include "../../Library/math/GlobalRand.h"
#include "../../Library/math/RandomWalk.hpp"
#include "../../Interface/LogOutput/LogUtility.h"

using libra::Vector;
using libra::NormalRand;

using namespace libra;

Atmosphere::Atmosphere(string model, string fname, double gauss_stddev)   //コンストラクタ，空気密度外乱の割合を決定
{
  model_ = model;
  fname_ = fname;
  gauss_stddev_ = gauss_stddev;
  air_density_ = 0.0;
  is_table_imported_ = false;

  if (model_ == "STANDARD")
  {
    cerr << "Air density model : STANDARD" << endl;
  }
  else if (model_ == "NRLMSISE00")
  {
    cerr << "Air density model : NRLMSISE00" << endl;
  }
  else
  {
    cerr << "Air density model : None" << endl;
    cerr << "Air density is set as 0.0 kg/m3" << endl;
  }
}

int Atmosphere::GetSpaceWeatherTable(double decyear, double endsec)
{
  // メモリ節約のためシミュレーション期間のみのテーブルを取得
  return GetSpaceWeatherTable_(decyear, endsec, fname_, table_);
}

double Atmosphere::GetAirDensity() const //値を返すだけの関数
{
  return air_density_;
}

double Atmosphere::CalcAirDensity(double decyear, double endsec, Vector<3> lat_lon_alt)
{
  if (!IsCalcEnabled) return 0;

  if (model_ == "STANDARD")
  {
    double altitude_m = lat_lon_alt(2);
    air_density_ = CalcStandard(altitude_m);
  }
  else if (model_ == "NRLMSISE00") // NRLMSISE00 model
  {
    if (!is_table_imported_)
    {
      if (GetSpaceWeatherTable(decyear, endsec))
      {
        is_table_imported_ = true;
      }
      else
      {
        cerr << "Air density is changed to STANDARD model" << endl;
        model_ = "STANDARD";
      }
    }

    double latrad = lat_lon_alt(0);
    double lonrad = lat_lon_alt(1);
    double alt = lat_lon_alt(2);
    air_density_ = CalcNRLMSISE00(decyear, latrad, lonrad, alt, table_);
  }
  else
  {
    // 該当のモデルがない場合は0を返す
    return air_density_ = 0.0;
  }

  return AddNoise(air_density_);
}

double Atmosphere::CalcStandard(double altitude_m)
{
  //altitude_mの単位は[m]
  double altitude = altitude_m / 1000.0; //kmへ変換
  double scaleHeight;//[km]
  double baseHeight;//[km]
  double baseRho;//[kg/m^3]

  //scaleHeightなどの値は，『ミッション解析と軌道設計の基礎』より
  if (altitude > 1000.0)
  {
    scaleHeight = 268.0;
    baseHeight = 1000.0;
    baseRho = 3.019E-15;
  }
  else if (altitude >= 900.0 && altitude < 1000.0)
  {
    scaleHeight = 181.05;
    baseHeight = 900.0;
    baseRho = 5.245E-15;
  }
  else if (altitude >= 800.0 && altitude < 900.0)
  {
    scaleHeight = 124.64;
    baseHeight = 800.0;
    baseRho = 1.170E-14;
  }
  else if (altitude >= 700.0 && altitude < 800.0)
  {
    scaleHeight = 88.667;
    baseHeight = 700.0;
    baseRho = 3.614E-14;
  }
  else if (altitude >= 600.0 && altitude < 700.0)
  {
    scaleHeight = 71.835;
    baseHeight = 600.0;
    baseRho = 1.454E-13;
  }
  else if (altitude >= 500.0 && altitude < 600.0)
  {
    scaleHeight = 63.822;
    baseHeight = 500.0;
    baseRho = 6.967E-13;
  }
  else if (altitude >= 450.0 && altitude < 500.0)
  {
    scaleHeight = 60.828;
    baseHeight = 450.0;
    baseRho = 1.585E-12;
  }
  else if (altitude >= 400.0 && altitude < 450.0)
  {
    scaleHeight = 58.515;
    baseHeight = 400.0;
    baseRho = 3.725E-12;
  }
  else if (altitude >= 350.0 && altitude < 400.0)
  {
    scaleHeight = 53.298;
    baseHeight = 350.0;
    baseRho = 9.158E-12;
  }
  else if (altitude >= 300.0 && altitude < 350.0)
  {
    scaleHeight = 53.628;
    baseHeight = 300.0;
    baseRho = 2.418E-11;
  }
  else if (altitude >= 250.0 && altitude < 300.0)
  {
    scaleHeight = 45.546;
    baseHeight = 250.0;
    baseRho = 7.248E-11;
  }
  else if (altitude >= 200.0 && altitude < 250.0)
  {
    scaleHeight = 37.105;
    baseHeight = 200.0;
    baseRho = 2.789E-10;
  }
  else if (altitude >= 180.0 && altitude < 200.0)
  {
    scaleHeight = 29.740;
    baseHeight = 180.0;
    baseRho = 5.464E-10;
  }
  else if (altitude >= 150.0 && altitude < 180.0)
  {
    scaleHeight = 22.523;
    baseHeight = 150.0;
    baseRho = 2.070E-9;
  }
  else if (altitude >= 140.0 && altitude < 150.0)
  {
    scaleHeight = 16.149;
    baseHeight = 140.0;
    baseRho = 3.845E-9;
  }
  else if (altitude >= 130.0 && altitude < 140.0)
  {
    scaleHeight = 12.636;
    baseHeight = 130.0;
    baseRho = 8.484E-9;
  }
  else if (altitude >= 120.0 && altitude < 130.0)
  {
    scaleHeight = 9.473;
    baseHeight = 120.0;
    baseRho = 2.438E-8;
  }
  else if (altitude >= 110.0 && altitude < 120.0)
  {
    scaleHeight = 7.263;
    baseHeight = 110.0;
    baseRho = 9.661E-8;
  }
  else if (altitude >= 100.0 && altitude < 110.0)
  {
    scaleHeight = 5.877;
    baseHeight = 100.0;
    baseRho = 5.297E-7;
  }
  else if (altitude >= 90.0 && altitude < 100.0)
  {
    scaleHeight = 5.382;
    baseHeight = 90.0;
    baseRho = 3.396E-6;
  }
  else if (altitude >= 80.0 && altitude < 90.0)
  {
    scaleHeight = 5.799;
    baseHeight = 80.0;
    baseRho = 1.905E-5;
  }
  else if (altitude >= 70.0 && altitude < 80.0)
  {
    scaleHeight = 6.549;
    baseHeight = 70.0;
    baseRho = 8.770E-5;
  }
  else if (altitude >= 60.0 && altitude < 70.0)
  {
    scaleHeight = 7.714;
    baseHeight = 60.0;
    baseRho = 3.206E-4;
  }
  else if (altitude >= 50.0 && altitude < 60.0)
  {
    scaleHeight = 8.382;
    baseHeight = 50.0;
    baseRho = 1.057E-3;
  }
  else if (altitude >= 40.0 && altitude < 50.0)
  {
    scaleHeight = 7.554;
    baseHeight = 40.0;
    baseRho = 3.972E-3;
  }
  else if (altitude >= 30.0 && altitude < 40.0)
  {
    scaleHeight = 6.682;
    baseHeight = 30.0;
    baseRho = 1.774E-2;
  }
  else if (altitude >= 25.0 && altitude < 30.0)
  {
    scaleHeight = 6.349;
    baseHeight = 25.0;
    baseRho = 3.899E-2;
  }
  else if (altitude >= 0.0 && altitude < 25.0)
  {
    scaleHeight = 7.249;
    baseHeight = 0.0;
    baseRho = 1.225;
  }
  else
  { //高度がマイナスの場合，0を返して終了
    scaleHeight = 7.249;
    baseHeight = 0.0;
    baseRho = 0.0;
    return 0.0;
  }

  double rho = baseRho * exp(-(altitude - baseHeight) / scaleHeight);
  return rho;
}

double Atmosphere::AddNoise(double rho) {
  //RandomWalk rw(rho*rw_stepwidth_,rho*rw_stddev_,rho*rw_limit_);
  NormalRand nr(0.0, rho*gauss_stddev_, g_rand.MakeSeed());
  double nrd = nr;

  return rho + nrd;
}

string Atmosphere::GetLogValue() const
{
  string str_tmp = "";
  str_tmp += WriteScalar(air_density_);

  return str_tmp;
}

string Atmosphere::GetLogHeader() const
{
  string str_tmp = "";

  str_tmp += WriteScalar("airdensity", "kg/m^3");

  return str_tmp;
}
