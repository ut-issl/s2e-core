/**
 * @file time_series_file_orbit_propagation.cpp
 * @brief Class to calculate satellite orbit using interpolation with orbit time series input
 */

#include "time_series_file_orbit_propagation.hpp"

#include <SpiceUsr.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <utilities/macros.hpp>

#include "logger/log_utility.hpp"
#include "math_physics/math/constants.hpp"
#include "math_physics/time_system/date_time_format.hpp"
#include "setting_file_reader/initialize_file_access.hpp"

TimeSeriesFileOrbitPropagation::TimeSeriesFileOrbitPropagation(const CelestialInformation* celestial_information, std::string time_series_file_path,
                                                               int number_of_interpolation, int interpolation_method,
                                                               double orbital_period_correction_s, const double current_time_jd)
    : Orbit(celestial_information) {
  propagate_mode_ = OrbitPropagateMode::kTimeSeriesFile;

  number_of_interpolation_ = number_of_interpolation;
  interpolation_method_ = interpolation_method;
  orbital_period_correction_s_ = orbital_period_correction_s;

  // Read time series file
  IniAccess time_series_file(time_series_file_path);
  time_series_file.ReadCsvDoubleWithHeader(time_series_data_, 7, 1, 0);
  int year, month, day, hour, minute;
  double second;
  for (size_t i = 0; i < time_series_data_.size(); ++i) {
    if (!time_series_data_[i].empty()) {
      SpiceChar utc_char[80];
      et2utc_c(time_series_data_[i][0], "ISOC", 2, 80, utc_char);
      sscanf(utc_char, "%d-%d-%dT%d:%d:%lf", &year, &month, &day, &hour, &minute, &second);
      epoch_.push_back(time_system::DateTime(year, month, day, hour, minute, second));
    }
  }

  // Get general info
  size_t nearest_epoch_id = SearchNearestEpochId(current_time_jd);

  const size_t half_interpolation_number = number_of_interpolation / 2;
  if (nearest_epoch_id >= half_interpolation_number) {
    reference_interpolation_id_ = nearest_epoch_id - half_interpolation_number;
  }

  reference_time_ = time_system::EpochTime(CalcEpochData(reference_interpolation_id_));

  // Initialize orbit
  orbit_position_i_m_.assign(1.0, orbit::InterpolationOrbit(number_of_interpolation));
  orbit_velocity_i_m_s_.assign(1.0, orbit::InterpolationOrbit(number_of_interpolation));

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

size_t TimeSeriesFileOrbitPropagation::SearchNearestEpochId(const double current_time_jd) {
  size_t nearest_epoch_id = 0;

  // Get start ephemeris time
  double start_ephemris_time = (current_time_jd - 2451545.0) * 86400.0;

  // Get the nearest epoch ID
  for (size_t i = 0; i < time_series_data_.size(); i++) {
    if (start_ephemris_time < time_series_data_[i][0]) {
      nearest_epoch_id = i;
      break;
    }
  }
  return nearest_epoch_id;
}

time_system::DateTime TimeSeriesFileOrbitPropagation::CalcEpochData(const size_t epoch_id) const {
  if (epoch_id > epoch_.size()) {
    time_system::DateTime zero;
    return zero;
  }
  return epoch_[epoch_id];
}

void TimeSeriesFileOrbitPropagation::Propagate(const double end_time_s, const double current_time_jd) {
  UNUSED(end_time_s);

  if (!is_calc_enabled_) return;

  // Get time
  int year, month, day, hour, minute;
  double second;
  SpiceChar current_utc_char[80];
  double current_ephemris_time = (current_time_jd - 2451545.0) * 86400.0;
  et2utc_c(current_ephemris_time, "ISOC", 2, 80, current_utc_char);
  sscanf(current_utc_char, "%d-%d-%dT%d:%d:%lf", &year, &month, &day, &hour, &minute, &second);
  time_system::DateTime current_date_time(year, month, day, hour, minute, second);
  current_epoch_time_ = time_system::EpochTime(current_date_time);

  // Check interpolation update
  double diff_s = current_epoch_time_.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
  double medium_time_s = orbit_position_i_m_[0].GetTimeList()[std::round(number_of_interpolation_ / 2.0)];
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
  time_system::EpochTime time_series_time = time_system::EpochTime(CalcEpochData(reference_interpolation_id_));
  double time_diff_s = time_series_time.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
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
