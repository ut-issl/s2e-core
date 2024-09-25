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

void TimeSeriesFileOrbitPropagation::Initialize(const std::vector<OrbitDefinitionData>& orbit_definition_data, const time_system::EpochTime start_time, const SimulationTime& simulation_time) {
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


  orbit_definition_data_ = orbit_definition_data;
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

std::vector<std::string> ParseCsvLine(const std::string& line) {
  std::vector<std::string> result;
  std::stringstream ss(line);
  std::string field;
  bool inside_quotes = false;
  char current_char;
  std::string temp;

  while (ss.get(current_char)) {
    if (current_char == '"') {
      inside_quotes = !inside_quotes;
      temp += '"';
    } else if (current_char == ',' && !inside_quotes) {
      result.push_back(temp);
      temp.clear();
    } else {
      temp += current_char;
    }
  }
  result.push_back(temp);

  return result;
}

bool TimeSeriesFileOrbitPropagation::ReadOrbitDefinitionCsv(const std::string ini_file_name, const std::string& orbit_definition_file_path, std::vector<OrbitDefinitionData>& orbit_definition_data) {
  ini_file_name_ = ini_file_name;
  IniAccess ini_file(ini_file_name_);
  char section[] = "TIME_SERIES_FILE_ORBIT_PROPAGATION";

  std::ifstream file(orbit_definition_file_path);
  if (!file.is_open()) {
    std::cout << "File open error. filename = " << orbit_definition_file_path << std::endl;
    is_calc_enabled_ = false;
    is_log_enabled_ = false;
    return false;
  }

  std::string line;

  if (std::getline(file, line)) {
    std::vector<std::string> headers = ParseCsvLine(line);

    int index_et = -1;
    int index_x = -1;
    int index_y = -1;
    int index_z = -1;
    int index_vx = -1;
    int index_vy = -1;
    int index_vz = -1;
    for (int i = 0; i < headers.size(); ++i) {
      if (headers[i] == ini_file.ReadString(section, "header_name_et")) {
        index_et = i;
      } else if (headers[i] == ini_file.ReadString(section, "header_name_x")) {
        index_x = i;
      } else if (headers[i] == ini_file.ReadString(section, "header_name_y")) {
        index_y = i;
      } else if (headers[i] == ini_file.ReadString(section, "header_name_z")) {
        index_z = i;
      } else if (headers[i] == ini_file.ReadString(section, "header_name_vx")) {
        index_vx = i;
      } else if (headers[i] == ini_file.ReadString(section, "header_name_vy")) {
        index_vy = i;
      } else if (headers[i] == ini_file.ReadString(section, "header_name_vz")) {
        index_vz = i;
      }
    }

    if (index_et == -1 || index_x == -1 || index_y == -1 || index_z == -1 || index_vx == -1 || index_vy == -1 || index_vz == -1) {
      std::cout << "Header name error." << std::endl;
      is_calc_enabled_ = false;
      is_log_enabled_ = false;
      return false;
    }

    int year, month, day, hour, minute;
    double second;

    while (std::getline(file, line)) {
      std::vector<std::string> row = ParseCsvLine(line);
      if (index_et < row.size() && index_x < row.size() && index_y < row.size() && index_z < row.size() && index_vx < row.size() && index_vy < row.size() && index_vz < row.size()) {
        OrbitDefinitionData data;
        data.et = std::stod(row[index_et]);
        data.x = std::stod(row[index_x]);
        data.y = std::stod(row[index_y]);
        data.z = std::stod(row[index_z]);
        data.vx = std::stod(row[index_vx]);
        data.vy = std::stod(row[index_vy]);
        data.vz = std::stod(row[index_vz]);

        SpiceChar utc_char[80];
        et2utc_c(data.et, "ISOC", 2, 80, utc_char);
        sscanf(utc_char, "%d-%d-%dT%d:%d:%lf", &year, &month, &day, &hour, &minute, &second);
        epoch_.push_back(time_system::DateTime(year, month, day, hour, minute, second));

        orbit_definition_data.push_back(data);
      }
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
  for (size_t i = 0; i < orbit_definition_data_.size(); i++) {
    if (start_et < orbit_definition_data_[i].et) {
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
    time_system::EpochTime orbit_definition_time = time_system::EpochTime(GetEpochData(reference_interpolation_id_));
    double time_diff_s = orbit_definition_time.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
    math::Vector<3> orbit_definition_position;
    math::Vector<3> orbit_definition_velocity;
    orbit_definition_position(0) = orbit_definition_data_[reference_interpolation_id_].x;
    orbit_definition_position(1) = orbit_definition_data_[reference_interpolation_id_].y;
    orbit_definition_position(2) = orbit_definition_data_[reference_interpolation_id_].z;
    orbit_definition_velocity(0) = orbit_definition_data_[reference_interpolation_id_].vx;
    orbit_definition_velocity(1) = orbit_definition_data_[reference_interpolation_id_].vy;
    orbit_definition_velocity(2) = orbit_definition_data_[reference_interpolation_id_].vz;

    orbit_position_[0].PushAndPopData(time_diff_s, orbit_definition_position);
    orbit_velocity_[0].PushAndPopData(time_diff_s, orbit_definition_velocity);
  
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

  const std::string orbit_definition_file_path = ini_file.ReadString(section, "orbit_definition_file_path");

  std::vector<OrbitDefinitionData> orbit_definition_data;
  if (!time_series_file_orbit_propagation->ReadOrbitDefinitionCsv(ini_file_name, orbit_definition_file_path, orbit_definition_data)) {
    return time_series_file_orbit_propagation;
  }

  time_system::DateTime start_date_time((size_t)simulation_time.GetStartYear(), (size_t)simulation_time.GetStartMonth(),
                                        (size_t)simulation_time.GetStartDay(), (size_t)simulation_time.GetStartHour(),
                                        (size_t)simulation_time.GetStartMinute(), simulation_time.GetStartSecond());
  time_system::EpochTime start_epoch_time(start_date_time);
  time_series_file_orbit_propagation->Initialize(orbit_definition_data, start_epoch_time, simulation_time);

  return time_series_file_orbit_propagation;
}
