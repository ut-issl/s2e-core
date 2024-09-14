/**
 * @file orbit.cpp
 * @brief Base class of orbit propagation
 */
#include "orbit.hpp"

s2e::math::Quaternion Orbit::CalcQuaternion_i2lvlh() const {
  s2e::math::Vector<3> lvlh_x = spacecraft_position_i_m_;  // x-axis in LVLH frame is position vector direction from geocenter to satellite
  s2e::math::Vector<3> lvlh_ex = lvlh_x.CalcNormalizedVector();
  s2e::math::Vector<3> lvlh_z =
      OuterProduct(spacecraft_position_i_m_, spacecraft_velocity_i_m_s_);  // z-axis in LVLH frame is angular momentum vector direction of orbit
  s2e::math::Vector<3> lvlh_ez = lvlh_z.CalcNormalizedVector();
  s2e::math::Vector<3> lvlh_y = OuterProduct(lvlh_z, lvlh_x);
  s2e::math::Vector<3> lvlh_ey = lvlh_y.CalcNormalizedVector();

  s2e::math::Matrix<3, 3> dcm_i2lvlh;
  dcm_i2lvlh[0][0] = lvlh_ex[0];
  dcm_i2lvlh[0][1] = lvlh_ex[1];
  dcm_i2lvlh[0][2] = lvlh_ex[2];
  dcm_i2lvlh[1][0] = lvlh_ey[0];
  dcm_i2lvlh[1][1] = lvlh_ey[1];
  dcm_i2lvlh[1][2] = lvlh_ey[2];
  dcm_i2lvlh[2][0] = lvlh_ez[0];
  dcm_i2lvlh[2][1] = lvlh_ez[1];
  dcm_i2lvlh[2][2] = lvlh_ez[2];

  s2e::math::Quaternion q_i2lvlh = s2e::math::Quaternion::ConvertFromDcm(dcm_i2lvlh);
  return q_i2lvlh.Normalize();
}

void Orbit::TransformEciToEcef(void) {
  s2e::math::Matrix<3, 3> dcm_i_to_xcxf = celestial_information_->GetEarthRotation().GetDcmJ2000ToEcef();
  spacecraft_position_ecef_m_ = dcm_i_to_xcxf * spacecraft_position_i_m_;

  // convert velocity vector in ECI to the vector in ECEF
  s2e::math::Vector<3> earth_angular_velocity_i_rad_s{0.0};
  earth_angular_velocity_i_rad_s[2] = environment::earth_mean_angular_velocity_rad_s;
  s2e::math::Vector<3> we_cross_r = OuterProduct(earth_angular_velocity_i_rad_s, spacecraft_position_i_m_);
  s2e::math::Vector<3> velocity_we_cross_r = spacecraft_velocity_i_m_s_ - we_cross_r;
  spacecraft_velocity_ecef_m_s_ = dcm_i_to_xcxf * velocity_we_cross_r;
}

void Orbit::TransformEcefToGeodetic(void) {
  spacecraft_geodetic_position_.UpdateFromEcef(spacecraft_position_ecef_m_);
  // Check altitude
  if (spacecraft_geodetic_position_.GetAltitude_m() < 0.0) {
    std::cout << "[Error Orbit]: The spacecraft altitude is smaller than zero." << std::endl;
    std::cout << "               The orbit or disturbance setting may have something wrong." << std::endl;
    std::exit(1);
  }
}

OrbitInitializeMode SetOrbitInitializeMode(const std::string initialize_mode) {
  if (initialize_mode == "DEFAULT") {
    return OrbitInitializeMode::kDefault;
  } else if (initialize_mode == "POSITION_VELOCITY_I") {
    return OrbitInitializeMode::kInertialPositionAndVelocity;
  } else if (initialize_mode == "ORBITAL_ELEMENTS") {
    return OrbitInitializeMode::kOrbitalElements;
  } else {
    std::cerr << "WARNINGS: orbit initialize mode is not defined!" << std::endl;
    std::cerr << "The orbit is automatically initialized as default mode" << std::endl;
    return OrbitInitializeMode::kDefault;
  }
}

std::string Orbit::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("spacecraft_position", "i", "m", 3);
  str_tmp += WriteVector("spacecraft_position", "ecef", "m", 3);
  str_tmp += WriteVector("spacecraft_velocity", "i", "m/s", 3);
  str_tmp += WriteVector("spacecraft_velocity", "b", "m/s", 3);
  str_tmp += WriteVector("spacecraft_acceleration", "i", "m/s2", 3);
  str_tmp += WriteScalar("spacecraft_latitude", "rad");
  str_tmp += WriteScalar("spacecraft_longitude", "rad");
  str_tmp += WriteScalar("spacecraft_altitude", "m");

  return str_tmp;
}

std::string Orbit::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(spacecraft_position_i_m_, 16);
  str_tmp += WriteVector(spacecraft_position_ecef_m_, 16);
  str_tmp += WriteVector(spacecraft_velocity_i_m_s_, 10);
  str_tmp += WriteVector(spacecraft_velocity_b_m_s_, 10);
  str_tmp += WriteVector(spacecraft_acceleration_i_m_s2_, 10);
  str_tmp += WriteScalar(spacecraft_geodetic_position_.GetLatitude_rad());
  str_tmp += WriteScalar(spacecraft_geodetic_position_.GetLongitude_rad());
  str_tmp += WriteScalar(spacecraft_geodetic_position_.GetAltitude_m());

  return str_tmp;
}
