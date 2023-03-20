/**
 * @file sgp4_orbit_propagation.cpp
 * @brief Class to propagate spacecraft orbit with SGP4 method with TLE
 */

#include "sgp4_orbit_propagation.hpp"

#include <iostream>
#include <library/utilities/macros.hpp>
#include <sstream>

Sgp4OrbitPropagation::Sgp4OrbitPropagation(const CelestialInformation* celestial_information, char* tle1, char* tle2, const int wgs_setting,
                                           const double current_time_jd)
    : Orbit(celestial_information) {
  propagate_mode_ = OrbitPropagateMode::kSgp4;

  if (wgs_setting == 0) {
    gravity_constant_setting_ = wgs72old;
  } else if (wgs_setting == 1) {
    gravity_constant_setting_ = wgs72;
  } else if (wgs_setting == 2) {
    gravity_constant_setting_ = wgs84;
  }

  char type_run = 'c', type_input = 0;
  double start_mfe, stop_mfe, delta_min;

  twoline2rv(tle1, tle2, type_run, type_input, gravity_constant_setting_, start_mfe, stop_mfe, delta_min, sgp4_data_);

  spacecraft_acceleration_i_m_s2_ *= 0.0;

  // To calculate initial position and velocity
  is_calc_enabled_ = true;
  Propagate(0.0, current_time_jd);
  is_calc_enabled_ = false;
}

void Sgp4OrbitPropagation::Propagate(const double end_time_s, const double current_time_jd) {
  UNUSED(end_time_s);

  if (!is_calc_enabled_) return;
  double elapse_time_min = (current_time_jd - sgp4_data_.jdsatepoch) * (24.0 * 60.0);

  double position_i_km[3];
  double velocity_i_km_s[3];

  sgp4(gravity_constant_setting_, sgp4_data_, elapse_time_min, position_i_km, velocity_i_km_s);

  // Error in SGP4
  if (sgp4_data_.error > 0) printf("# *** error: time:= %f *** code = %3d\n", sgp4_data_.t, sgp4_data_.error);

  for (int i = 0; i < 3; ++i) {
    spacecraft_position_i_m_[i] = position_i_km[i] * 1000.0;
    spacecraft_velocity_i_m_s_[i] = velocity_i_km_s[i] * 1000.0;
  }

  TransformEciToEcef();
  TransformEcefToGeodetic();
}
