/**
 * @file initialize_orbit.cpp
 * @brief Initialize function for Orbit class
 */
#include "initialize_orbit.hpp"

#include <setting_file_reader/initialize_file_access.hpp>

#include "encke_orbit_propagation.hpp"
#include "kepler_orbit_propagation.hpp"
#include "relative_orbit.hpp"
#include "rk4_orbit_propagation.hpp"
#include "sgp4_orbit_propagation.hpp"
#include "time_series_file_orbit_propagation.hpp"

namespace s2e::dynamics::orbit {

Orbit* InitOrbit(const environment::CelestialInformation* celestial_information, std::string initialize_file, double step_width_s,
                 double current_time_jd, double gravity_constant_m3_s2, std::string section, simulation::RelativeInformation* relative_information) {
  auto conf = setting_file_reader::IniAccess(initialize_file);
  const char* section_ = section.c_str();
  Orbit* orbit;

  // Initialize mode
  OrbitInitializeMode initialize_mode = SetOrbitInitializeMode(conf.ReadString(section_, "initialize_mode"));

  // Propagate mode
  std::string propagate_mode = conf.ReadString(section_, "propagate_mode");

  if (propagate_mode == "RK4") {
    // initialize RK4 orbit propagator
    math::Vector<3> position_i_m;
    math::Vector<3> velocity_i_m_s;
    math::Vector<6> pos_vel = InitializePosVel(initialize_file, current_time_jd, gravity_constant_m3_s2);
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
    s2e::orbit::RelativeOrbitModel relative_dynamics_model_type =
        (s2e::orbit::RelativeOrbitModel)(conf.ReadInt(section_, "relative_dynamics_model_type"));
    s2e::orbit::StmModel stm_model_type = (s2e::orbit::StmModel)(conf.ReadInt(section_, "stm_model_type"));

    math::Vector<3> init_relative_position_lvlh;
    conf.ReadVector<3>(section_, "initial_relative_position_lvlh_m", init_relative_position_lvlh);
    math::Vector<3> init_relative_velocity_lvlh;
    conf.ReadVector<3>(section_, "initial_relative_velocity_lvlh_m_s", init_relative_velocity_lvlh);

    // There is a possibility that the orbit of the reference sat is not initialized when RelativeOrbit initialization is called To ensure that
    // the orbit of the reference sat is initialized, create temporary initial orbit of the reference sat
    int reference_spacecraft_id = conf.ReadInt(section_, "reference_satellite_id");

    orbit = new RelativeOrbit(celestial_information, gravity_constant_m3_s2, step_width_s, reference_spacecraft_id, init_relative_position_lvlh,
                              init_relative_velocity_lvlh, update_method, relative_dynamics_model_type, stm_model_type, relative_information);
  } else if (propagate_mode == "KEPLER") {
    // initialize orbit for Kepler propagation
    s2e::orbit::OrbitalElements oe;
    // TODO: init_mode_kepler should be removed in the next major update
    if (initialize_mode == OrbitInitializeMode::kInertialPositionAndVelocity) {
      // initialize with position and velocity
      math::Vector<3> init_pos_m;
      conf.ReadVector<3>(section_, "initial_position_i_m", init_pos_m);
      math::Vector<3> init_vel_m_s;
      conf.ReadVector<3>(section_, "initial_velocity_i_m_s", init_vel_m_s);
      oe = s2e::orbit::OrbitalElements(gravity_constant_m3_s2, current_time_jd, init_pos_m, init_vel_m_s);
    } else {
      // initialize with orbital elements
      double semi_major_axis_m = conf.ReadDouble(section_, "semi_major_axis_m");
      double eccentricity = conf.ReadDouble(section_, "eccentricity");
      double inclination_rad = conf.ReadDouble(section_, "inclination_rad");
      double raan_rad = conf.ReadDouble(section_, "raan_rad");
      double arg_perigee_rad = conf.ReadDouble(section_, "argument_of_perigee_rad");
      double epoch_jday = conf.ReadDouble(section_, "epoch_jday");
      oe = s2e::orbit::OrbitalElements(epoch_jday, semi_major_axis_m, eccentricity, inclination_rad, raan_rad, arg_perigee_rad);
    }
    s2e::orbit::KeplerOrbit kepler_orbit(gravity_constant_m3_s2, oe);
    orbit = new KeplerOrbitPropagation(celestial_information, current_time_jd, kepler_orbit);
  } else if (propagate_mode == "ENCKE") {
    // initialize orbit for Encke's method
    math::Vector<3> position_i_m;
    math::Vector<3> velocity_i_m_s;
    math::Vector<6> pos_vel = InitializePosVel(initialize_file, current_time_jd, gravity_constant_m3_s2);
    for (size_t i = 0; i < 3; i++) {
      position_i_m[i] = pos_vel[i];
      velocity_i_m_s[i] = pos_vel[i + 3];
    }

    double error_tolerance = conf.ReadDouble(section_, "error_tolerance");
    orbit = new EnckeOrbitPropagation(celestial_information, gravity_constant_m3_s2, step_width_s, current_time_jd, position_i_m, velocity_i_m_s,
                                      error_tolerance);
  } else if (propagate_mode == "TIME_SERIES_FILE") {
    // initialize orbit for propagation with time series file
    const std::string time_series_file_path = conf.ReadString(section_, "time_series_file_path");
    const int number_of_interpolation = conf.ReadInt(section_, "number_of_interpolation");
    const int interpolation_method = conf.ReadInt(section_, "interpolation_method");
    const double orbital_period_correction_s = conf.ReadDouble(section_, "orbital_period_correction_s");

    orbit = new TimeSeriesFileOrbitPropagation(celestial_information, time_series_file_path, number_of_interpolation, interpolation_method,
                                               orbital_period_correction_s, current_time_jd);

  } else {
    std::cerr << "ERROR: orbit propagation mode: " << propagate_mode << " is not defined!" << std::endl;
    std::cerr << "The orbit mode is automatically set as RK4" << std::endl;

    math::Vector<3> position_i_m;
    math::Vector<3> velocity_i_m_s;
    math::Vector<6> pos_vel = InitializePosVel(initialize_file, current_time_jd, gravity_constant_m3_s2);
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

math::Vector<6> InitializePosVel(std::string initialize_file, double current_time_jd, double gravity_constant_m3_s2, std::string section) {
  auto conf = setting_file_reader::IniAccess(initialize_file);
  const char* section_ = section.c_str();
  math::Vector<3> position_i_m;
  math::Vector<3> velocity_i_m_s;
  math::Vector<6> pos_vel;

  OrbitInitializeMode initialize_mode = SetOrbitInitializeMode(conf.ReadString(section_, "initialize_mode"));
  if (initialize_mode == OrbitInitializeMode::kOrbitalElements) {
    double semi_major_axis_m = conf.ReadDouble(section_, "semi_major_axis_m");
    double eccentricity = conf.ReadDouble(section_, "eccentricity");
    double inclination_rad = conf.ReadDouble(section_, "inclination_rad");
    double raan_rad = conf.ReadDouble(section_, "raan_rad");
    double arg_perigee_rad = conf.ReadDouble(section_, "argument_of_perigee_rad");
    double epoch_jday = conf.ReadDouble(section_, "epoch_jday");
    s2e::orbit::OrbitalElements oe(epoch_jday, semi_major_axis_m, eccentricity, inclination_rad, raan_rad, arg_perigee_rad);
    s2e::orbit::KeplerOrbit kepler_orbit(gravity_constant_m3_s2, oe);

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

}  // namespace s2e::dynamics::orbit
