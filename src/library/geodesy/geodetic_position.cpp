/**
 * @file geodetic_position.cpp
 * @brief Class to mange geodetic position expression
 */
#include "geodetic_position.hpp"

#include <library/sgp4/sgp4ext.h>  // TODO: do not to use the functions in SGP4 library

#include <library/math/Constant.hpp>
#include <library/math/Matrix.hpp>
#include <environment/global/physical_constants.hpp>

GeodeticPosition::GeodeticPosition() {
  latitude_rad_ = 0.0;
  longitude_rad_ = 0.0;
  altitude_m_ = 0.0;
  CalcQuaternionXcxfToLtc();
}

GeodeticPosition::GeodeticPosition(const double latitude_rad, const double longitude_rad, const double altitude_m)
    : latitude_rad_(latitude_rad), longitude_rad_(longitude_rad), altitude_m_(altitude_m) {
  // TODO: Add assertion check for altitude limit
  CalcQuaternionXcxfToLtc();
}

void GeodeticPosition::UpdateFromEcef(const libra::Vector<3> position_ecef_m) {
  const double earth_radius_m = environment::earth_equatorial_radius_m;
  const double flattening = environment::earth_flattening;

  double r_m, e2, phi, c;
  double theta_rad;

  theta_rad = AcTan(position_ecef_m[1], position_ecef_m[0]);
  longitude_rad_ = FMod2p(theta_rad);
  r_m = sqrt(position_ecef_m[0] * position_ecef_m[0] + position_ecef_m[1] * position_ecef_m[1]);
  e2 = flattening * (2.0 - flattening);
  double lat_tmp_rad = AcTan(position_ecef_m[2], r_m);

  do {
    phi = lat_tmp_rad;
    c = 1.0 / sqrt(1.0 - e2 * sin(phi) * sin(phi));
    lat_tmp_rad = AcTan(position_ecef_m[2] + earth_radius_m * c * e2 * sin(phi), r_m);
  } while (fabs(lat_tmp_rad - phi) >= 1E-10);

  altitude_m_ = r_m / cos(lat_tmp_rad) - c * earth_radius_m;

  if (lat_tmp_rad > libra::pi_2) lat_tmp_rad -= libra::tau;

  latitude_rad_ = lat_tmp_rad;

  CalcQuaternionXcxfToLtc();
  return;
}

libra::Vector<3> GeodeticPosition::CalcEcefPosition() const {
  const double earth_radius_m = environment::earth_equatorial_radius_m;
  const double flattening = environment::earth_flattening;

  double theta = FMod2p(longitude_rad_);
  double e2 = flattening * (2.0 - flattening);
  double c = 1 / sqrt(1.0 - e2 * sin(latitude_rad_) * sin(latitude_rad_));
  double N = c * earth_radius_m;

  libra::Vector<3> pos_ecef_m;
  pos_ecef_m(0) = (N + altitude_m_) * cos(latitude_rad_) * cos(theta);
  pos_ecef_m(1) = (N + altitude_m_) * cos(latitude_rad_) * sin(theta);
  pos_ecef_m(2) = (N * (1 - e2) + altitude_m_) * sin(latitude_rad_);

  return pos_ecef_m;
}

void GeodeticPosition::CalcQuaternionXcxfToLtc() {
  libra::Matrix<3, 3> trans_mat_xcxf_to_ltc;
  trans_mat_xcxf_to_ltc[0][0] = -sin(longitude_rad_);
  trans_mat_xcxf_to_ltc[0][1] = cos(longitude_rad_);
  trans_mat_xcxf_to_ltc[0][2] = 0;
  trans_mat_xcxf_to_ltc[1][0] = -sin(latitude_rad_) * cos(longitude_rad_);
  trans_mat_xcxf_to_ltc[1][1] = -sin(latitude_rad_) * sin(longitude_rad_);
  trans_mat_xcxf_to_ltc[1][2] = cos(latitude_rad_);
  trans_mat_xcxf_to_ltc[2][0] = cos(latitude_rad_) * cos(longitude_rad_);
  trans_mat_xcxf_to_ltc[2][1] = cos(latitude_rad_) * sin(longitude_rad_);
  trans_mat_xcxf_to_ltc[2][2] = sin(latitude_rad_);

  q_xcxf_to_ltc_ = q_xcxf_to_ltc_.fromDCM(trans_mat_xcxf_to_ltc);
}
