/**
 * @file time_series_file_orbit_propagation.cpp
 * @brief Class to calculate satellite orbit using interpolation with orbit time series input
 */

#include "time_series_file_orbit_propagation.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <utilities/macros.hpp>

#include "logger/log_utility.hpp"
#include "math_physics/math/constants.hpp"
#include "setting_file_reader/initialize_file_access.hpp"

TimeSeriesFileOrbitPropagation::TimeSeriesFileOrbitPropagation(const CelestialInformation* celestial_information, std::string time_series_file_path,
                                                               int number_of_interpolation, int interpolation_method,
                                                               double orbital_period_correction_s, const double current_time_jd)
    : Orbit(celestial_information),
      is_time_range_warning_displayed_(false),
      is_interpolation_method_error_displayed_(false),
      number_of_interpolation_(number_of_interpolation),
      interpolation_method_(interpolation_method),
      orbital_period_correction_s_(orbital_period_correction_s),
      reference_interpolation_id_(0) {
  propagate_mode_ = OrbitPropagateMode::kTimeSeriesFile;

  // Read time series file
  IniAccess time_series_file(time_series_file_path);
  time_series_file.ReadCsvDoubleWithHeader(time_series_data_, 7, 1, 0);

  // Get general info
  size_t nearest_ephemeris_time_id = SearchNearestEphemerisTimeId(current_time_jd);

  const size_t half_interpolation_number = number_of_interpolation / 2;
  if (nearest_ephemeris_time_id >= half_interpolation_number) {
    reference_interpolation_id_ = nearest_ephemeris_time_id - half_interpolation_number;
  }

  reference_time_ = CalcEphemerisTimeData(reference_interpolation_id_);

  // Initialize orbit
  orbit_position_i_m_.assign(1, orbit::InterpolationOrbit(number_of_interpolation));
  orbit_velocity_i_m_s_.assign(1, orbit::InterpolationOrbit(number_of_interpolation));

  // Initialize interpolation
  for (int i = 0; i < number_of_interpolation; i++) {
    UpdateInterpolationInformation();
  }
  spacecraft_acceleration_i_m_s2_ *= 0.0;

  // To calculate initial position and velocity
  is_calc_enabled_ = true;
  Propagate(0.0, current_time_jd);
  is_calc_enabled_ = false;
}

size_t TimeSeriesFileOrbitPropagation::SearchNearestEphemerisTimeId(const double current_time_jd) {
  size_t nearest_ephemeris_time_id = 0;

  // Get start ephemeris time
  double start_ephemris_time = (current_time_jd - 2451545.0) * 86400.0;

  // Get the nearest ephemeris time ID
  for (size_t i = 0; i < time_series_data_.size(); i++) {
    if (start_ephemris_time < time_series_data_[i][0]) {
      nearest_ephemeris_time_id = i;
      break;
    }
  }
  return nearest_ephemeris_time_id;
}

double TimeSeriesFileOrbitPropagation::CalcEphemerisTimeData(const size_t ephemeris_time_id) const {
  if (ephemeris_time_id > time_series_data_.size()) {
    return 0;
  }
  return time_series_data_[ephemeris_time_id][0];
}

void TimeSeriesFileOrbitPropagation::Propagate(const double end_time_s, const double current_time_jd) {
  UNUSED(end_time_s);

  if (!is_calc_enabled_) return;

  // Check interpolation update
  double current_ephemris_time = (current_time_jd - 2451545.0) * 86400.0;
  double diff_s = current_ephemris_time - reference_time_;
  double medium_time_s = orbit_position_i_m_[0].GetTimeList()[static_cast<std::size_t>(std::round(number_of_interpolation_ / 2.0))];
  if (diff_s > medium_time_s) {
    UpdateInterpolationInformation();
  }
  if (diff_s < 0.0) {
    if (!is_time_range_warning_displayed_) {
      std::cout << "[WARNING] Time series file orbit propagation: Time is out of range of time series file." << std::endl;
      is_time_range_warning_displayed_ = true;
    }
    for (size_t i = 0; i < 3; i++) {
      spacecraft_position_i_m_[i] = time_series_data_[0][i + 1];
      spacecraft_velocity_i_m_s_[i] = time_series_data_[0][i + 4];
    }
  } else if (diff_s > 1e6 || reference_interpolation_id_ >= time_series_data_.size()) {
    if (!is_time_range_warning_displayed_) {
      std::cout << "[WARNING] Time series file orbit propagation: Time is out of range of time series file." << std::endl;
      is_time_range_warning_displayed_ = true;
    }
    for (size_t i = 0; i < 3; i++) {
      spacecraft_position_i_m_[i] = time_series_data_[time_series_data_.size() - 1][i + 1];
      spacecraft_velocity_i_m_s_[i] = time_series_data_[time_series_data_.size() - 1][i + 4];
    }
  } else {
    if (interpolation_method_ == 0) {
      spacecraft_position_i_m_ = orbit_position_i_m_[0].CalcPositionWithPolynomial(diff_s);
      spacecraft_velocity_i_m_s_ = orbit_velocity_i_m_s_[0].CalcPositionWithPolynomial(diff_s);
    } else if (interpolation_method_ == 1) {
      spacecraft_position_i_m_ = orbit_position_i_m_[0].CalcPositionWithTrigonometric(diff_s, math::tau / orbital_period_correction_s_);
      spacecraft_velocity_i_m_s_ = orbit_velocity_i_m_s_[0].CalcPositionWithTrigonometric(diff_s, math::tau / orbital_period_correction_s_);
    } else {
      if (!is_interpolation_method_error_displayed_) {
        std::cerr << "[ERROR] Time series file orbit propagation: Interpolation method " << interpolation_method_ << " is not defined!" << std::endl;
        std::cerr << "The orbit mode is automatically set as Polynomial" << std::endl;
        is_interpolation_method_error_displayed_ = true;
      }
      spacecraft_position_i_m_ = orbit_position_i_m_[0].CalcPositionWithPolynomial(diff_s);
      spacecraft_velocity_i_m_s_ = orbit_velocity_i_m_s_[0].CalcPositionWithPolynomial(diff_s);
    }
  }
  TransformEciToEcef();
  TransformEcefToGeodetic();
}

bool TimeSeriesFileOrbitPropagation::UpdateInterpolationInformation() {
  double time_series_data_time_s = CalcEphemerisTimeData(reference_interpolation_id_);
  double time_diff_s = time_series_data_time_s - reference_time_;
  if (reference_interpolation_id_ >= time_series_data_.size()) {
    for (size_t i = 0; i < 3; i++) {
      spacecraft_position_i_m_[i] = time_series_data_[time_series_data_.size() - 1][i + 1];
      spacecraft_velocity_i_m_s_[i] = time_series_data_[time_series_data_.size() - 1][i + 4];
    }
  } else {
    for (size_t i = 0; i < 3; i++) {
      spacecraft_position_i_m_[i] = time_series_data_[reference_interpolation_id_][i + 1];
      spacecraft_velocity_i_m_s_[i] = time_series_data_[reference_interpolation_id_][i + 4];
    }
  }

  orbit_position_i_m_[0].PushAndPopData(time_diff_s, spacecraft_position_i_m_);
  orbit_velocity_i_m_s_[0].PushAndPopData(time_diff_s, spacecraft_velocity_i_m_s_);

  reference_interpolation_id_++;

  return true;
}

