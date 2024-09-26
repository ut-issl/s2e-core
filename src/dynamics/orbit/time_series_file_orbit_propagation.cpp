/**
 * @file time_series_file_orbit_propagation.cpp
 * @brief Class to calculate satellite orbit using interpolation with orbit time series input
 */

#include "time_series_file_orbit_propagation.hpp"

#include <SpiceUsr.h>

#include <fstream>
#include <sstream>
#include <iostream>

#include "setting_file_reader/initialize_file_access.hpp"
#include "math_physics/time_system/date_time_format.hpp"
#include "math_physics/math/constants.hpp"
#include "logger/log_utility.hpp"

void TimeSeriesFileOrbitPropagation::Initialize(const std::string ini_file_name, const std::vector<std::vector<double>>& time_series_data, const time_system::EpochTime start_time, const SimulationTime& simulation_time) {
  ini_file_name_ = ini_file_name;
  IniAccess ini_file(ini_file_name_);
  char section[] = "TIME_SERIES_FILE_ORBIT_PROPAGATION";
  const size_t number_of_interpolation = ini_file.ReadInt(section, "number_of_interpolation");

  if (!(ini_file.ReadString(section, "interpolation_method") == "POLYNOMIAL" || 
      ini_file.ReadString(section, "interpolation_method") == "TRIGONOMETRIC")) {
    std::cout << "Interpolation method error." << std::endl;
    is_calc_enabled_ = false;
    is_log_enabled_ = false;
    return;
  }


  time_series_data_ = time_series_data;
  current_epoch_time_ = start_time;

  // Get general info
  const size_t nearest_epoch_id = SearchNearestEpochId(simulation_time);
  const size_t half_interpolation_number = number_of_interpolation / 2;
  if (nearest_epoch_id >= half_interpolation_number) {
    reference_interpolation_id_ = nearest_epoch_id - half_interpolation_number;
  }
  reference_time_ = time_system::EpochTime(GetEpochData(reference_interpolation_id_));

  // Initialize orbit
  orbit_position_.assign(1.0, orbit::InterpolationOrbit(number_of_interpolation));
  orbit_velocity_.assign(1.0, orbit::InterpolationOrbit(number_of_interpolation));

  // Initialize interpolation
  for (size_t i = 0; i < number_of_interpolation; i++) {
    UpdateInterpolationInformation();
  }
  return;
}

bool TimeSeriesFileOrbitPropagation::ReadTimeSeriesCsv(const std::string& time_series_file_path, std::vector<std::vector<double>>& time_series_data) {
  IniAccess time_series_file(time_series_file_path);


  time_series_file.ReadCsvDoubleWithHeader(time_series_data, 7, 1, 0);

  int year, month, day, hour, minute;
  double second;

  for (size_t i = 0; i < time_series_data.size(); ++i) {
    if (!time_series_data[i].empty()) {
      SpiceChar utc_char[80];
      et2utc_c(time_series_data[i][0], "ISOC", 2, 80, utc_char);
      sscanf(utc_char, "%d-%d-%dT%d:%d:%lf", &year, &month, &day, &hour, &minute, &second);
      epoch_.push_back(time_system::DateTime(year, month, day, hour, minute, second));
    }
  }
  return true;
}


void TimeSeriesFileOrbitPropagation::Update(const SimulationTime& simulation_time) {
  if (!IsCalcEnabled()) return;

  // Get time
  UTC current_utc = simulation_time.GetCurrentUtc();
  time_system::DateTime current_date_time((size_t)current_utc.year, (size_t)current_utc.month, (size_t)current_utc.day, (size_t)current_utc.hour,
                                          (size_t)current_utc.minute, current_utc.second);
  current_epoch_time_ = time_system::EpochTime(current_date_time);

  // Check interpolation update
  double diff_s = current_epoch_time_.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
  double medium_time_s = orbit_position_[0].GetTimeList()[4];
  if (diff_s > medium_time_s) {
    UpdateInterpolationInformation();
  }

  return;
}

size_t TimeSeriesFileOrbitPropagation::SearchNearestEpochId(const SimulationTime& simulation_time) {
  size_t nearest_epoch_id = 0;

  // Get start ephemeris time
  SpiceDouble start_et = simulation_time.GetCurrentEphemerisTime();

  // Get the nearest epoch ID
  for (size_t i = 0; i < time_series_data_.size(); i++) {
    if (start_et < time_series_data_[i][0]) {
      nearest_epoch_id = i;
      break;
    }
  }
  return nearest_epoch_id;
}

time_system::DateTime TimeSeriesFileOrbitPropagation::GetEpochData(const size_t epoch_id) const {
  if (epoch_id > epoch_.size()) {
    time_system::DateTime zero;
    return zero;
  }
  return epoch_[epoch_id];
}


math::Vector<3> TimeSeriesFileOrbitPropagation::GetPosition(const time_system::EpochTime time) const {
  IniAccess ini_file(ini_file_name_);
  char section[] = "TIME_SERIES_FILE_ORBIT_PROPAGATION";

  time_system::EpochTime target_time;

  if (time.GetTime_s() == 0) {
    target_time = current_epoch_time_;
  } else {
    target_time = time;
  }

  double diff_s = target_time.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
  if (diff_s < 0.0 || diff_s > 1e6) return math::Vector<3>(0.0);

  if (ini_file.ReadString(section, "interpolation_method") == "POLYNOMIAL") {
    return orbit_position_[0].CalcPositionOrVelocityWithPolynomial(diff_s);
  } else if (ini_file.ReadString(section, "interpolation_method") == "TRIGONOMETRIC") {
    return orbit_position_[0].CalcPositionOrVelocityWithTrigonometric(diff_s, math::tau / ini_file.ReadDouble(section, "orbital_period_correction"));
  } else {
    return math::Vector<3>(0.0);
  }
}

math::Vector<3> TimeSeriesFileOrbitPropagation::GetVelocity(const time_system::EpochTime time) const {
  IniAccess ini_file(ini_file_name_);
  char section[] = "TIME_SERIES_FILE_ORBIT_PROPAGATION";

  time_system::EpochTime target_time;

  if (time.GetTime_s() == 0) {
    target_time = current_epoch_time_;
  } else {
    target_time = time;
  }

  double diff_s = target_time.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
  if (diff_s < 0.0 || diff_s > 1e6) return math::Vector<3>(0.0);

  if (ini_file.ReadString(section, "interpolation_method") == "POLYNOMIAL") {
    return orbit_velocity_[0].CalcPositionOrVelocityWithPolynomial(diff_s);
  } else if (ini_file.ReadString(section, "interpolation_method") == "TRIGONOMETRIC") {
    return orbit_velocity_[0].CalcPositionOrVelocityWithTrigonometric(diff_s, math::tau / ini_file.ReadDouble(section, "orbital_period_correction"));
  } else {
    return math::Vector<3>(0.0);
  }
}

bool TimeSeriesFileOrbitPropagation::UpdateInterpolationInformation() {
    time_system::EpochTime time_series_time = time_system::EpochTime(GetEpochData(reference_interpolation_id_));
    double time_diff_s = time_series_time.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
    math::Vector<3> time_series_position;
    math::Vector<3> time_series_velocity;
    time_series_position[0] = time_series_data_[reference_interpolation_id_][1];
    time_series_position[1] = time_series_data_[reference_interpolation_id_][2];
    time_series_position[2] = time_series_data_[reference_interpolation_id_][3];
    time_series_velocity[0] = time_series_data_[reference_interpolation_id_][4];
    time_series_velocity[1] = time_series_data_[reference_interpolation_id_][5];
    time_series_velocity[2] = time_series_data_[reference_interpolation_id_][6];

    orbit_position_[0].PushAndPopData(time_diff_s, time_series_position);
    orbit_velocity_[0].PushAndPopData(time_diff_s, time_series_velocity);
  
  reference_interpolation_id_++;

  return true;
}

std::string TimeSeriesFileOrbitPropagation::GetLogHeader() const {
  IniAccess ini_file(ini_file_name_);
  char section[] = "TIME_SERIES_FILE_ORBIT_PROPAGATION";

  std::string str_tmp = "";

  str_tmp += WriteVector("spacecraft_position_with_definition_file", ini_file.ReadString(section, "coordinate_system"), ini_file.ReadString(section, "unit"), 3);
  str_tmp += WriteVector("spacecraft_velocity_with_definition_file", ini_file.ReadString(section, "coordinate_system"), ini_file.ReadString(section, "unit"), 3);

  return str_tmp;
}

std::string TimeSeriesFileOrbitPropagation::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(GetPosition(), 16);
  str_tmp += WriteVector(GetVelocity(), 16);

  return str_tmp;
}

TimeSeriesFileOrbitPropagation* InitTimeSeriesFileOrbitPropagation(const std::string ini_file_name, const SimulationTime& simulation_time) {
  IniAccess ini_file(ini_file_name);
  char section[] = "TIME_SERIES_FILE_ORBIT_PROPAGATION";

  const bool is_calc_enable = ini_file.ReadEnable(section, INI_CALC_LABEL);
  const bool is_log_enable = ini_file.ReadEnable(section, INI_LOG_LABEL);

  TimeSeriesFileOrbitPropagation* time_series_file_orbit_propagation = new TimeSeriesFileOrbitPropagation(is_calc_enable, is_log_enable);
  if (!time_series_file_orbit_propagation->IsCalcEnabled()) {
    return time_series_file_orbit_propagation;
  }

  const std::string time_series_file_path = ini_file.ReadString(section, "time_series_file_path");

  std::vector<std::vector<double>> time_series_data;
  if (!time_series_file_orbit_propagation->ReadTimeSeriesCsv(time_series_file_path, time_series_data)) {
    return time_series_file_orbit_propagation;
  }

  time_system::DateTime start_date_time((size_t)simulation_time.GetStartYear(), (size_t)simulation_time.GetStartMonth(),
                                        (size_t)simulation_time.GetStartDay(), (size_t)simulation_time.GetStartHour(),
                                        (size_t)simulation_time.GetStartMinute(), simulation_time.GetStartSecond());
  time_system::EpochTime start_epoch_time(start_date_time);
  time_series_file_orbit_propagation->Initialize(ini_file_name, time_series_data, start_epoch_time, simulation_time);

  return time_series_file_orbit_propagation;
}
