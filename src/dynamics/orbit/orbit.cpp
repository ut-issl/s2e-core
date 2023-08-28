/**
 * @file orbit.cpp
 * @brief Base class of orbit propagation
 */
#include "orbit.hpp"

#include <library/initialize/initialize_file_access.hpp>

#include "encke_orbit_propagation.hpp"
#include "kepler_orbit_propagation.hpp"
#include "relative_orbit.hpp"
#include "rk4_orbit_propagation.hpp"
#include "sgp4_orbit_propagation.hpp"

libra::Quaternion Orbit::CalcQuaternion_i2lvlh() const {
  libra::Vector<3> lvlh_x = spacecraft_position_i_m_;  // x-axis in LVLH frame is position vector direction from geocenter to satellite
  libra::Vector<3> lvlh_ex = lvlh_x.CalcNormalizedVector();
  libra::Vector<3> lvlh_z =
      OuterProduct(spacecraft_position_i_m_, spacecraft_velocity_i_m_s_);  // z-axis in LVLH frame is angular momentum vector direction of orbit
  libra::Vector<3> lvlh_ez = lvlh_z.CalcNormalizedVector();
  libra::Vector<3> lvlh_y = OuterProduct(lvlh_z, lvlh_x);
  libra::Vector<3> lvlh_ey = lvlh_y.CalcNormalizedVector();

  libra::Matrix<3, 3> dcm_i2lvlh;
  dcm_i2lvlh[0][0] = lvlh_ex[0];
  dcm_i2lvlh[0][1] = lvlh_ex[1];
  dcm_i2lvlh[0][2] = lvlh_ex[2];
  dcm_i2lvlh[1][0] = lvlh_ey[0];
  dcm_i2lvlh[1][1] = lvlh_ey[1];
  dcm_i2lvlh[1][2] = lvlh_ey[2];
  dcm_i2lvlh[2][0] = lvlh_ez[0];
  dcm_i2lvlh[2][1] = lvlh_ez[1];
  dcm_i2lvlh[2][2] = lvlh_ez[2];

  libra::Quaternion q_i2lvlh = libra::Quaternion::ConvertFromDcm(dcm_i2lvlh);
  return q_i2lvlh.Normalize();
}

void Orbit::TransformEciToEcef(void) {
  libra::Matrix<3, 3> dcm_i_to_xcxf = celestial_information_->GetEarthRotation().GetDcmJ2000ToXcxf();
  spacecraft_position_ecef_m_ = dcm_i_to_xcxf * spacecraft_position_i_m_;

  // convert velocity vector in ECI to the vector in ECEF
  libra::Vector<3> earth_angular_velocity_i_rad_s{0.0};
  earth_angular_velocity_i_rad_s[2] = environment::earth_mean_angular_velocity_rad_s;
  libra::Vector<3> we_cross_r = OuterProduct(earth_angular_velocity_i_rad_s, spacecraft_position_i_m_);
  libra::Vector<3> velocity_we_cross_r = spacecraft_velocity_i_m_s_ - we_cross_r;
  spacecraft_velocity_ecef_m_s_ = dcm_i_to_xcxf * velocity_we_cross_r;
}

void Orbit::TransformEcefToGeodetic(void) { spacecraft_geodetic_position_.UpdateFromEcef(spacecraft_position_ecef_m_); }

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
  str_tmp += WriteVector(spacecraft_velocity_i_m_s_, 10);
  str_tmp += WriteVector(spacecraft_velocity_b_m_s_, 10);
  str_tmp += WriteVector(spacecraft_acceleration_i_m_s2_, 10);
  str_tmp += WriteScalar(spacecraft_geodetic_position_.GetLatitude_rad());
  str_tmp += WriteScalar(spacecraft_geodetic_position_.GetLongitude_rad());
  str_tmp += WriteScalar(spacecraft_geodetic_position_.GetAltitude_m());

  return str_tmp;
}

Orbit* InitOrbit(const CelestialInformation* celestial_information, std::string initialize_file, double step_width_s, double current_time_jd,
                 double gravity_constant_m3_s2, std::string section, RelativeInformation* relative_information) {
  auto conf = IniAccess(initialize_file);
  const char* section_ = section.c_str();
  Orbit* orbit;

  // Initialize mode
  OrbitInitializeMode initialize_mode = SetOrbitInitializeMode(conf.ReadString(section_, "initialize_mode"));

  // Propagate mode
  std::string propagate_mode = conf.ReadString(section_, "propagate_mode");

  if (propagate_mode == "RK4") {
    // initialize RK4 orbit propagator
    libra::Vector<3> position_i_m;
    libra::Vector<3> velocity_i_m_s;
    libra::Vector<6> pos_vel = InitializePosVel(initialize_file, current_time_jd, gravity_constant_m3_s2);
    for (size_t i = 0; i < 3; i++) {
      position_i_m[i] = pos_vel[i];
      velocity_i_m_s[i] = pos_vel[i + 3];
    }
    orbit = new Rk4OrbitPropagation(celestial_information, gravity_constant_m3_s2, step_width_s, position_i_m, velocity_i_m_s);
  } else if (propagate_mode == "SGP4") {
    // Initialize SGP4 orbit propagator
    int wgs_setting = conf.ReadInt(section_, "wgs_setting");
    char tle1[80], tle2[80];
    conf.ReadChar(section_, "tle1", 80, tle1);
    conf.ReadChar(section_, "tle2", 80, tle2);

    orbit = new Sgp4OrbitPropagation(celestial_information, tle1, tle2, wgs_setting, current_time_jd);
  } else if (propagate_mode == "RELATIVE") {
    // initialize orbit for relative dynamics of formation flying
    RelativeOrbit::RelativeOrbitUpdateMethod update_method =
        (RelativeOrbit::RelativeOrbitUpdateMethod)(conf.ReadInt(section_, "relative_orbit_update_method"));
    RelativeOrbitModel relative_dynamics_model_type = (RelativeOrbitModel)(conf.ReadInt(section_, "relative_dynamics_model_type"));
    StmModel stm_model_type = (StmModel)(conf.ReadInt(section_, "stm_model_type"));

    libra::Vector<3> init_relative_position_lvlh;
    conf.ReadVector<3>(section_, "initial_relative_position_lvlh_m", init_relative_position_lvlh);
    libra::Vector<3> init_relative_velocity_lvlh;
    conf.ReadVector<3>(section_, "initial_relative_velocity_lvlh_m_s", init_relative_velocity_lvlh);

    // There is a possibility that the orbit of the reference sat is not initialized when RelativeOrbit initialization is called To ensure that
    // the orbit of the reference sat is initialized, create temporary initial orbit of the reference sat
    int reference_spacecraft_id = conf.ReadInt(section_, "reference_satellite_id");

    orbit = new RelativeOrbit(celestial_information, gravity_constant_m3_s2, step_width_s, reference_spacecraft_id, init_relative_position_lvlh,
                              init_relative_velocity_lvlh, update_method, relative_dynamics_model_type, stm_model_type, relative_information);
  } else if (propagate_mode == "KEPLER") {
    // initialize orbit for Kepler propagation
    OrbitalElements oe;
    // TODO: init_mode_kepler should be removed in the next major update
    if (initialize_mode == OrbitInitializeMode::kInertialPositionAndVelocity) {
      // initialize with position and velocity
      libra::Vector<3> init_pos_m;
      conf.ReadVector<3>(section_, "initial_position_i_m", init_pos_m);
      libra::Vector<3> init_vel_m_s;
      conf.ReadVector<3>(section_, "initial_velocity_i_m_s", init_vel_m_s);
      oe = OrbitalElements(gravity_constant_m3_s2, current_time_jd, init_pos_m, init_vel_m_s);
    } else {
      // initialize with orbital elements
      double semi_major_axis_m = conf.ReadDouble(section_, "semi_major_axis_m");
      double eccentricity = conf.ReadDouble(section_, "eccentricity");
      double inclination_rad = conf.ReadDouble(section_, "inclination_rad");
      double raan_rad = conf.ReadDouble(section_, "raan_rad");
      double arg_perigee_rad = conf.ReadDouble(section_, "argument_of_perigee_rad");
      double epoch_jday = conf.ReadDouble(section_, "epoch_jday");
      oe = OrbitalElements(epoch_jday, semi_major_axis_m, eccentricity, inclination_rad, raan_rad, arg_perigee_rad);
    }
    KeplerOrbit kepler_orbit(gravity_constant_m3_s2, oe);
    orbit = new KeplerOrbitPropagation(celestial_information, current_time_jd, kepler_orbit);
  } else if (propagate_mode == "ENCKE") {
    // initialize orbit for Encke's method
    libra::Vector<3> position_i_m;
    libra::Vector<3> velocity_i_m_s;
    libra::Vector<6> pos_vel = InitializePosVel(initialize_file, current_time_jd, gravity_constant_m3_s2);
    for (size_t i = 0; i < 3; i++) {
      position_i_m[i] = pos_vel[i];
      velocity_i_m_s[i] = pos_vel[i + 3];
    }

    double error_tolerance = conf.ReadDouble(section_, "error_tolerance");
    orbit = new EnckeOrbitPropagation(celestial_information, gravity_constant_m3_s2, step_width_s, current_time_jd, position_i_m, velocity_i_m_s,
                                      error_tolerance);
  } else {
    std::cerr << "ERROR: orbit propagation mode: " << propagate_mode << " is not defined!" << std::endl;
    std::cerr << "The orbit mode is automatically set as RK4" << std::endl;

    libra::Vector<3> position_i_m;
    libra::Vector<3> velocity_i_m_s;
    libra::Vector<6> pos_vel = InitializePosVel(initialize_file, current_time_jd, gravity_constant_m3_s2);
    for (size_t i = 0; i < 3; i++) {
      position_i_m[i] = pos_vel[i];
      velocity_i_m_s[i] = pos_vel[i + 3];
    }
    orbit = new Rk4OrbitPropagation(celestial_information, gravity_constant_m3_s2, step_width_s, position_i_m, velocity_i_m_s);
  }

  orbit->SetIsCalcEnabled(conf.ReadEnable(section_, "calculation"));
  orbit->is_log_enabled_ = conf.ReadEnable(section_, "logging");
  return orbit;
}

libra::Vector<6> InitializePosVel(std::string initialize_file, double current_time_jd, double gravity_constant_m3_s2, std::string section) {
  auto conf = IniAccess(initialize_file);
  const char* section_ = section.c_str();
  libra::Vector<3> position_i_m;
  libra::Vector<3> velocity_i_m_s;
  libra::Vector<6> pos_vel;

  OrbitInitializeMode initialize_mode = SetOrbitInitializeMode(conf.ReadString(section_, "initialize_mode"));
  if (initialize_mode == OrbitInitializeMode::kOrbitalElements) {
    double semi_major_axis_m = conf.ReadDouble(section_, "semi_major_axis_m");
    double eccentricity = conf.ReadDouble(section_, "eccentricity");
    double inclination_rad = conf.ReadDouble(section_, "inclination_rad");
    double raan_rad = conf.ReadDouble(section_, "raan_rad");
    double arg_perigee_rad = conf.ReadDouble(section_, "argument_of_perigee_rad");
    double epoch_jday = conf.ReadDouble(section_, "epoch_jday");
    OrbitalElements oe(epoch_jday, semi_major_axis_m, eccentricity, inclination_rad, raan_rad, arg_perigee_rad);
    KeplerOrbit kepler_orbit(gravity_constant_m3_s2, oe);

    kepler_orbit.CalcOrbit(current_time_jd);
    position_i_m = kepler_orbit.GetPosition_i_m();
    velocity_i_m_s = kepler_orbit.GetVelocity_i_m_s();
  } else {
    conf.ReadVector<3>(section_, "initial_position_i_m", position_i_m);
    conf.ReadVector<3>(section_, "initial_velocity_i_m_s", velocity_i_m_s);
  }

  for (size_t i = 0; i < 3; i++) {
    pos_vel[i] = position_i_m[i];
    pos_vel[i + 3] = velocity_i_m_s[i];
  }

  return pos_vel;
}
