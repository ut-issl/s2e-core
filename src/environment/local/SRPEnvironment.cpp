/**
 * @file SRPEnvironment.cpp
 * @brief Class to calculate Solar Radiation Pressure
 */
#include "SRPEnvironment.h"

#include <Interface/LogOutput/LogUtility.h>

#include <environment/global/PhysicalConstants.hpp>
#include <Library/math/Constant.hpp>
#include <Library/math/Vector.hpp>
#include <algorithm>
#include <cassert>
#include <fstream>

using libra::Vector;
using namespace std;

SRPEnvironment::SRPEnvironment(LocalCelestialInformation* local_celes_info) : local_celes_info_(local_celes_info) {
  solar_constant_ = 1366.0;                                       // [W/m2]
  pressure_ = solar_constant_ / environment::speed_of_light_m_s;  // [N/m2]
  shadow_source_name_ = local_celes_info_->GetGlobalInfo().GetCenterBodyName();
  sun_radius_m_ = local_celes_info_->GetGlobalInfo().GetMeanRadiusFromName("SUN");
}

void SRPEnvironment::UpdateAllStates() {
  if (!IsCalcEnabled) return;

  UpdatePressure();
  CalcShadowCoefficient(shadow_source_name_);
}

void SRPEnvironment::UpdatePressure() {
  const Vector<3> r_sc2sun_eci = local_celes_info_->GetPosFromSC_i("SUN");
  const double distance_sat_to_sun = norm(r_sc2sun_eci);
  pressure_ = solar_constant_ / environment::speed_of_light_m_s / pow(distance_sat_to_sun / environment::astronomical_unit_m, 2.0);
}

double SRPEnvironment::CalcTruePressure() const { return pressure_ * shadow_coefficient_; }

double SRPEnvironment::CalcPowerDensity() const { return pressure_ * environment::speed_of_light_m_s * shadow_coefficient_; }

double SRPEnvironment::GetPressure() const { return pressure_; }

double SRPEnvironment::GetSolarConstant() const { return solar_constant_; }

double SRPEnvironment::GetShadowCoefficient() const { return shadow_coefficient_; }

string SRPEnvironment::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteScalar("solar_radiation_pressure_at_spacecraft_position", "N/m2");
  str_tmp += WriteScalar("shadow_coefficient_at_spacecraft_position");

  return str_tmp;
}

string SRPEnvironment::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteScalar(pressure_ * shadow_coefficient_);
  str_tmp += WriteScalar(shadow_coefficient_);

  return str_tmp;
}

void SRPEnvironment::CalcShadowCoefficient(string shadow_source_name) {
  if (shadow_source_name == "SUN") {
    shadow_coefficient_ = 1.0;
    return;
  }

  const Vector<3> r_sc2sun_eci = local_celes_info_->GetPosFromSC_i("SUN");
  const Vector<3> r_sc2source_eci = local_celes_info_->GetPosFromSC_i(shadow_source_name.c_str());

  const double shadow_source_radius_m = local_celes_info_->GetGlobalInfo().GetMeanRadiusFromName(shadow_source_name.c_str());

  const double distance_sat_to_sun = norm(r_sc2sun_eci);
  const double sd_sun = asin(sun_radius_m_ / distance_sat_to_sun);                // Apparent radius of the sun
  const double sd_source = asin(shadow_source_radius_m / norm(r_sc2source_eci));  // Apparent radius of the shadow source

  // Angle of deviation from shadow source center to sun center
  const double delta =
      acos(inner_product(r_sc2source_eci, r_sc2sun_eci - r_sc2source_eci) / norm(r_sc2source_eci) / norm(r_sc2sun_eci - r_sc2source_eci));
  // The angle between the center of the sun and the common chord
  const double x = (delta * delta + sd_sun * sd_sun - sd_source * sd_source) / (2.0 * delta);
  // The length of the common chord of the apparent solar disk and apparent telestial disk
  const double y = sqrt(max(sd_sun * sd_sun - x * x, 0.0));

  const double a = sd_sun;
  const double b = sd_source;
  const double c = delta;

  if (c < fabs(a - b) && a <= b)  // The occultation is total (spacecraft is in umbra)
  {
    shadow_coefficient_ = 0.0;
  } else if (c < fabs(a - b) && a > b)  // The occultation is partial but maximum
  {
    shadow_coefficient_ = 1.0 - (b * b) / (a * a);
  } else if (fabs(a - b) <= c && c <= (a + b))  // spacecraft is in penumbra
  {
    double A = a * a * acos(x / a) + b * b * acos((c - x) / b) - c * y;  // The area of the occulted segment of the apparent solar disk
    shadow_coefficient_ = 1.0 - A / (libra::pi * a * a);
  } else {  // no occultation takes place
    assert(c > (a + b));
    shadow_coefficient_ = 1.0;
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
        ep[0] = 6400000 * cos(libra::pi * i / 180);
        ep[1] = 6400000 * sin(libra::pi * i / 180);
        srp.Update(ep,sp);
        log2 << srp.GetP() << ",";
    }
}*/
