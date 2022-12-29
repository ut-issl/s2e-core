/**
 * @file Sgp4OrbitPropagation.cpp
 * @brief Class to propagate spacecraft orbit with SGP4 method with TLE
 */

#include "Sgp4OrbitPropagation.h"

#include <Library/utils/Macros.hpp>
#include <iostream>
#include <sstream>
using namespace std;

Sgp4OrbitPropagation::Sgp4OrbitPropagation(const CelestialInformation* celes_info, char* tle1, char* tle2, int wgs, double current_jd)
    : Orbit(celes_info) {
  propagate_mode_ = OrbitPropagateMode::SGP4;

  if (wgs == 0) {
    whichconst_ = wgs72old;
  } else if (wgs == 1) {
    whichconst_ = wgs72;
  } else if (wgs == 2) {
    whichconst_ = wgs84;
  }

  char typerun = 'c', typeinput = 0;
  double startmfe, stopmfe, deltamin;

  twoline2rv(tle1, tle2, typerun, typeinput, whichconst_, startmfe, stopmfe, deltamin, satrec_);

  acc_i_ *= 0;

  // To calculate initial position and velocity
  is_calc_enabled_ = true;
  Propagate(0.0, current_jd);
  is_calc_enabled_ = false;
}

void Sgp4OrbitPropagation::Propagate(double endtime, double current_jd) {
  UNUSED(endtime);

  if (!is_calc_enabled_) return;
  double elapse_time_min = (current_jd - satrec_.jdsatepoch) * (24.0 * 60.0);

  double r[3];
  double v[3];

  sgp4(whichconst_, satrec_, elapse_time_min, r, v);

  // Error in SGP4
  if (satrec_.error > 0) printf("# *** error: time:= %f *** code = %3d\n", satrec_.t, satrec_.error);

  for (int i = 0; i < 3; ++i) {
    sat_position_i_[i] = r[i] * 1000;
    sat_velocity_i_[i] = v[i] * 1000;
  }

  TransEciToEcef();
  TransEcefToGeo();
}

string Sgp4OrbitPropagation::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("sat_position", "i", "m", 3);
  str_tmp += WriteVector("sat_velocity", "i", "m/s", 3);
  str_tmp += WriteVector("sat_velocity", "b", "m/s", 3);
  str_tmp += WriteVector("sat_acc_i", "i", "m/s^2", 3);
  str_tmp += WriteScalar("lat", "rad");
  str_tmp += WriteScalar("lon", "rad");
  str_tmp += WriteScalar("alt", "m");

  return str_tmp;
}

string Sgp4OrbitPropagation::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(sat_position_i_, 16);
  str_tmp += WriteVector(sat_velocity_i_, 10);
  str_tmp += WriteVector(sat_velocity_b_, 10);
  str_tmp += WriteVector(acc_i_, 10);
  str_tmp += WriteScalar(sat_position_geo_.GetLat_rad());
  str_tmp += WriteScalar(sat_position_geo_.GetLon_rad());
  str_tmp += WriteScalar(sat_position_geo_.GetAlt_m());

  return str_tmp;
}

Vector<3> Sgp4OrbitPropagation::GetESIOmega() {
  Vector<3> omega_peri = Vector<3>();
  omega_peri[0] = 0.0;
  omega_peri[1] = 0.0;
  omega_peri[2] = satrec_.no / 60;

  double i = satrec_.inclo;      // inclination
  double OMEGA = satrec_.nodeo;  // raan
  double omega = satrec_.argpo;  // argment of perigee

  double comega = cos(omega);
  double cOMEGA = cos(OMEGA);
  double ci = cos(i);
  double somega = sin(omega);
  double sOMEGA = sin(OMEGA);
  double si = sin(i);

  Matrix<3, 3> PERI2ECI = Matrix<3, 3>();
  PERI2ECI[0][0] = comega * cOMEGA - somega * ci * sOMEGA;
  PERI2ECI[1][0] = -1.0 * somega * cOMEGA - 1.0 * comega * ci * sOMEGA;
  PERI2ECI[2][0] = si * sOMEGA;
  PERI2ECI[0][1] = comega * sOMEGA + somega * ci * cOMEGA;
  PERI2ECI[1][1] = -1.0 * somega * sOMEGA + comega * ci * cOMEGA;
  PERI2ECI[2][1] = -1.0 * si * cOMEGA;
  PERI2ECI[0][2] = somega * si;
  PERI2ECI[1][2] = comega * si;
  PERI2ECI[2][2] = ci;

  return PERI2ECI * omega_peri;
}
