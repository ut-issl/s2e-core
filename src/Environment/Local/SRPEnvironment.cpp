#include "SRPEnvironment.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <cassert>
#include <algorithm>
#include <Library/math/Vector.hpp>
#include <Interface/LogOutput/LogUtility.h>

using libra::Vector;
using namespace std;

SRPEnvironment::SRPEnvironment(LocalCelestialInformation* local_celes_info, string shadow_source_name):local_celes_info_(local_celes_info), shadow_source_name_(shadow_source_name)
{
  astronomical_unit_ = 149597870700.0;//[m]
  c_ = 299792458.0;                   //[m/s]
  solar_constant_ = 1366.0;           //[W/m2]
  pressure_ = solar_constant_ / c_;   //[N/m2]
  sun_radius_m_ = local_celes_info_->GetGlobalInfo().GetMeanRadiusFromName("SUN");
  source_radius_m_ = local_celes_info_->GetGlobalInfo().GetMeanRadiusFromName(shadow_source_name_.c_str());
}

void SRPEnvironment::UpdateAllStates()
{
  if (!IsCalcEnabled) return;

  UpdatePressure();
  CalcShadowFunction();
}

void SRPEnvironment::UpdatePressure()
{
  const Vector<3> r_sc2sun_eci = local_celes_info_->GetPosFromSC_i("SUN");
  const double distance_sat_to_sun = norm(r_sc2sun_eci);
  pressure_ = solar_constant_ / c_ / pow(distance_sat_to_sun / astronomical_unit_, 2.0);
}

double SRPEnvironment::CalcTruePressure() const
{
  return pressure_ * shadow_function_;
}

double SRPEnvironment::CalcPowerDensity() const
{
  return pressure_ * c_ * shadow_function_;
}

double SRPEnvironment::GetPressure() const
{
  return pressure_;
}

double SRPEnvironment::GetSolarConstant() const
{
  return solar_constant_;
}

double SRPEnvironment::GetShadowFunction() const
{
  return shadow_function_;
}


string SRPEnvironment::GetLogHeader() const
{
  string str_tmp = "";

  str_tmp += WriteScalar("sr_pressure", "N/m^2");
  str_tmp += WriteScalar("shadow function");

  return str_tmp;
}

string SRPEnvironment::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteScalar(pressure_ * shadow_function_);
  str_tmp += WriteScalar(shadow_function_);

  return str_tmp;
}

void SRPEnvironment::CalcShadowFunction()
{
  const Vector<3> r_sc2sun_eci = local_celes_info_->GetPosFromSC_i("SUN");
  const Vector<3> r_sc2source_eci = local_celes_info_->GetPosFromSC_i(shadow_source_name_.c_str());

  const double distance_sat_to_sun = norm(r_sc2sun_eci);
  const double sd_sun = asin(sun_radius_m_ / distance_sat_to_sun);         //Apparent radius of the sun
  const double sd_source = asin(source_radius_m_ / norm(r_sc2source_eci)); //Apparent radius of the shadow source

  const double delta = acos(inner_product(r_sc2source_eci, r_sc2sun_eci - r_sc2source_eci) / norm(r_sc2source_eci) / norm(r_sc2sun_eci - r_sc2source_eci));//Angle of deviation from shadow source center to sun center
  const double x = (delta * delta + sd_sun * sd_sun - sd_source * sd_source) / (2.0 * delta); //The angle between the center of the sun and the common chord
  const double y = sqrt(max(sd_sun * sd_sun - x * x, 0.0)); //The length of the common chord of the apparent solar disk and apparent tellestial disk
  const double a = sd_sun;
  const double b = sd_source;
  const double c = delta;

  if (c < fabs(a - b) && a <= b) //The occultation is total (spacecraft is in umbra)
  {
    shadow_function_ = 0;
  }
  else if (c < fabs(a - b) && a > b) //The occultation is partial but maximum
  {
    shadow_function_ = 1.0 - (b * b) / (a * a);
  }
  else if (fabs(a - b) <= c && c <= (a + b)) // spacecraft is in penunbra
  {
    double A = a * a * acos(x / a) + b * b * acos((c - x) / b) - c * y;//The area of the occulted segment of the apparent solar disk
    shadow_function_ = 1.0 - A / (M_PI * a *a);
  }
  else {// no occultation takes place
    assert(c > (a + b));
    shadow_function_ = 1.0;
  }
  
}

/*int main(){
    Vector<3> ep;
    Vector<3> sp;
    ofstream log1;//ファイル書き込み用
    log1.open("log1.csv",ios::trunc);
    ofstream log2;//ファイル書き込み用
    log2.open("log2.csv",ios::trunc);
    sp[0] = 149597870700.0;
    sp[1] = 0;
    sp[2] = 0;
    ep[0] = 6400000;
    ep[1] = 0;
    ep[2] = 0;
    SRPEnvironment srp;
    for(long i = 6400000;i < 4200000000; i = i + 10000000){
        sp[0] = 149597870700.0 + double(i);
        ep[0] = double(i);
        srp.Update(ep,sp);
        log1 << srp.GetP() << ",";
    }
    for(int i = 0;i < 360; i++){
        ep[0] = 6400000 * cos(M_PI * i / 180);
        ep[1] = 6400000 * sin(M_PI * i / 180);
        srp.Update(ep,sp);
        log2 << srp.GetP() << ",";
    }
}*/
