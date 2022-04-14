#include "Orbit.h"

Quaternion Orbit::CalcQuaternionI2LVLH() const {
  Vector<3> lvlh_x = sat_position_i_;  // x-axis in LVLH frame is position vector direction
                                       // from geocenter to satellite
  Vector<3> lvlh_ex = normalize(lvlh_x);
  Vector<3> lvlh_z = outer_product(sat_position_i_,
                                   sat_velocity_i_);  // z-axis in LVLH frame is angular
                                                      // momentum vector direction of orbit
  Vector<3> lvlh_ez = normalize(lvlh_z);
  Vector<3> lvlh_y = outer_product(lvlh_z, lvlh_x);
  Vector<3> lvlh_ey = normalize(lvlh_y);

  Matrix<3, 3> dcm_i2lvlh;
  dcm_i2lvlh[0][0] = lvlh_ex[0];
  dcm_i2lvlh[0][1] = lvlh_ex[1];
  dcm_i2lvlh[0][2] = lvlh_ex[2];
  dcm_i2lvlh[1][0] = lvlh_ey[0];
  dcm_i2lvlh[1][1] = lvlh_ey[1];
  dcm_i2lvlh[1][2] = lvlh_ey[2];
  dcm_i2lvlh[2][0] = lvlh_ez[0];
  dcm_i2lvlh[2][1] = lvlh_ez[1];
  dcm_i2lvlh[2][2] = lvlh_ez[2];

  Quaternion q_i2lvlh = Quaternion::fromDCM(dcm_i2lvlh);
  return q_i2lvlh.normalize();
}

void Orbit::TransECIToECEF(void) {
  Matrix<3, 3> dcm_i_to_xcxf = celes_info_->GetEarthRotation().GetDCMJ2000toXCXF();
  sat_position_ecef_ = dcm_i_to_xcxf * sat_position_i_;

  // convert velocity vector in ECI to the vector in ECEF
  Vector<3> OmegaE{0.0};
  OmegaE[2] = environment::earth_mean_angular_velocity_rad_s;
  Vector<3> wExr = outer_product(OmegaE, sat_position_i_);
  Vector<3> V_wExr = sat_velocity_i_ - wExr;
  sat_velocity_ecef_ = dcm_i_to_xcxf * V_wExr;
}

void Orbit::TransEcefToGeo(void) {
  Vector<3> lla = celes_info_->GetEarthRotation().TransformEcefToGeo(sat_position_ecef_);
  lat_rad_ = lla[0];
  lon_rad_ = lla[1];
  alt_m_ = lla[2];
}
