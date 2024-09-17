/**
 * @file orbit_calculation_with_definition_file.cpp
 * @brief Class to calculate satellite orbit using interpolation with orbit time series input
 */

#include "orbit_calculation_with_definition_file.hpp"

#include <SpiceUsr.h>

#include <fstream>
#include <sstream>
#include <iostream>

#include "setting_file_reader/initialize_file_access.hpp"
#include "math_physics/time_system/date_time_format.hpp"
#include "math_physics/math/constants.hpp"
#include "logger/log_utility.hpp"

const size_t kNumberOfInterpolation = 9;

bool OrbitCalculationWithDefinitionFile::ReadOrbitDefinitionCsv(const std::string& file_name, std::vector<OrbitDefinitionData>& orbit_definition_data, char delimiter) {
  std::ifstream ifs(file_name);
  if (!ifs.is_open()) {
    std::cerr << "File open error: " << file_name << std::endl;
    return false;
  }

  std::string line;
  std::getline(ifs, line);  // Skip header

  while (std::getline(ifs, line)) {
    std::istringstream streamline(line);
    std::string cell;

    int year, day, hour, minute;
    double second;
    char month_str[4];

    int month_num;

    OrbitDefinitionData data;

    // Read time (UTC)
    std::getline(streamline, cell, delimiter);
    data.time_utc = cell;

    sscanf(cell.c_str(), "%d %3s %2d %2d:%2d:%lf", &year, month_str, &day, &hour, &minute, &second);

    if (strcmp(month_str, "JAN") == 0) month_num = 1;
    else if (strcmp(month_str, "FEB") == 0) month_num = 2;
    else if (strcmp(month_str, "MAR") == 0) month_num = 3;
    else if (strcmp(month_str, "APR") == 0) month_num = 4;
    else if (strcmp(month_str, "MAY") == 0) month_num = 5;
    else if (strcmp(month_str, "JUN") == 0) month_num = 6;
    else if (strcmp(month_str, "JUL") == 0) month_num = 7;
    else if (strcmp(month_str, "AUG") == 0) month_num = 8;
    else if (strcmp(month_str, "SEP") == 0) month_num = 9;
    else if (strcmp(month_str, "OCT") == 0) month_num = 10;
    else if (strcmp(month_str, "NOV") == 0) month_num = 11;
    else if (strcmp(month_str, "DEC") == 0) month_num = 12;
    else {
      std::cout << "Invalid month abbreviation: " << month_str << std::endl;
      return false;
    }
    epoch_.push_back(time_system::DateTime(year, month_num, day, hour, minute, second));

    // Read Ephemeris time [s]
    std::getline(streamline, cell, delimiter);
    data.time_et_s = std::stod(cell);

    // Read EQU X(ECLIPJ2000) [km]
    std::getline(streamline, cell, delimiter);
    data.equ_x_eclipj2000_km = std::stod(cell);

    // Read EQU Y(ECLIPJ2000) [km]
    std::getline(streamline, cell, delimiter);
    data.equ_y_eclipj2000_km = std::stod(cell);

    // Read EQU Z(ECLIPJ2000) [km]
    std::getline(streamline, cell, delimiter);
    data.equ_z_eclipj2000_km = std::stod(cell);

    // Read VX [km/s]
    std::getline(streamline, cell, delimiter);
    data.vx_eclipj2000_km_s = std::stod(cell);

    // Read VY [km/s]
    std::getline(streamline, cell, delimiter);
    data.vy_eclipj2000_km_s = std::stod(cell);

    // Read VZ [km/s]
    std::getline(streamline, cell, delimiter);
    data.vz_eclipj2000_km_s = std::stod(cell);

    // Read Mass [kg]
    std::getline(streamline, cell, delimiter);
    data.m_kg = std::stod(cell);

    // Read FX [kN]
    std::getline(streamline, cell, delimiter);
    data.fx_kN = std::stod(cell);

    // Read FY [kN]
    std::getline(streamline, cell, delimiter);
    data.fy_kN = std::stod(cell);

    // Read FZ [kN]
    std::getline(streamline, cell, delimiter);
    data.fz_kN = std::stod(cell);

    // Read DV NODE
    std::getline(streamline, cell, delimiter);
    data.dv_node = (cell == "true");

    // Read DV_X [km/s]
    std::getline(streamline, cell, delimiter);
    data.dv_x_eclipj2000_km_s = std::stod(cell);

    // Read DV_Y [km/s]
    std::getline(streamline, cell, delimiter);
    data.dv_y_eclipj2000_km_s = std::stod(cell);

    // Read DV_Z [km/s]
    std::getline(streamline, cell, delimiter);
    data.dv_z_eclipj2000_km_s = std::stod(cell);

    // Read SUN X (ECLIPJ2000) [km]
    std::getline(streamline, cell, delimiter);
    data.sun_x_eclipj2000_km = std::stod(cell);

    // Read SUN Y (ECLIPJ2000) [km]
    std::getline(streamline, cell, delimiter);
    data.sun_y_eclipj2000_km = std::stod(cell);

    // Read SUN Z (ECLIPJ2000) [km]
    std::getline(streamline, cell, delimiter);
    data.sun_z_eclipj2000_km = std::stod(cell);

    // Read MOON X (ECLIPJ2000) [km]
    std::getline(streamline, cell, delimiter);
    data.moon_x_eclipj2000_km = std::stod(cell);

    // Read MOON Y (ECLIPJ2000) [km]
    std::getline(streamline, cell, delimiter);
    data.moon_y_eclipj2000_km = std::stod(cell);

    // Read MOON Z (ECLIPJ2000) [km]
    std::getline(streamline, cell, delimiter);
    data.moon_z_eclipj2000_km = std::stod(cell);

    orbit_definition_data.push_back(data);
  }
  return true;
}

void OrbitCalculationWithDefinitionFile::Initialize(const std::vector<OrbitDefinitionData>& orbit_definition_data, const time_system::EpochTime start_time, const SimulationTime& simulation_time) {
  orbit_definition_data_ = orbit_definition_data;
  current_epoch_time_ = start_time;

  // Get the initialize SP3 file
  // OrbitDefinitionData initial_orbit_definition_data = orbit_definition_data_[0];

  // Get general info
  // number_of_calculated_gnss_satellites_ = initial_sp3_file.GetNumberOfSatellites();
  const size_t nearest_epoch_id = SearchNearestEpochId(simulation_time);
  const size_t half_interpolation_number = kNumberOfInterpolation / 2;
  if (nearest_epoch_id >= half_interpolation_number) {
    reference_interpolation_id_ = nearest_epoch_id - half_interpolation_number;
  }
  reference_time_ = time_system::EpochTime(GetEpochData(reference_interpolation_id_));

  // Initialize orbit
  orbit_.assign(1.0, orbit::InterpolationOrbit(kNumberOfInterpolation));

  // Initialize interpolation
  for (size_t i = 0; i < kNumberOfInterpolation; i++) {
    UpdateInterpolationInformation();
  }

  return;
}

void OrbitCalculationWithDefinitionFile::Update(const SimulationTime& simulation_time) {
  if (!IsCalcEnabled()) return;

  // Get time
  UTC current_utc = simulation_time.GetCurrentUtc();
  time_system::DateTime current_date_time((size_t)current_utc.year, (size_t)current_utc.month, (size_t)current_utc.day, (size_t)current_utc.hour,
                                          (size_t)current_utc.minute, current_utc.second);
  current_epoch_time_ = time_system::EpochTime(current_date_time);

  // Check interpolation update
  double diff_s = current_epoch_time_.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
  double medium_time_s = orbit_[0].GetTimeList()[4];
  if (diff_s > medium_time_s) {
    UpdateInterpolationInformation();
  }

  return;
}

size_t OrbitCalculationWithDefinitionFile::SearchNearestEpochId(const SimulationTime& simulation_time) {
  size_t nearest_epoch_id = 0;

  // Get start ephemeris time
  SpiceDouble start_et = simulation_time.GetCurrentEphemerisTime();

  // Get the nearest epoch ID
  for (size_t i = 0; i < orbit_definition_data_.size(); i++) {
    if (start_et < orbit_definition_data_[i].time_et_s) {
      nearest_epoch_id = i;
      break;
    }
  }
  return nearest_epoch_id;
}

time_system::DateTime OrbitCalculationWithDefinitionFile::GetEpochData(const size_t epoch_id) const {
  if (epoch_id > epoch_.size()) {
    time_system::DateTime zero;
    return zero;
  }
  return epoch_[epoch_id];
}


math::Vector<3> OrbitCalculationWithDefinitionFile::GetPosition_eclipj2000_km(const time_system::EpochTime time) const {
  time_system::EpochTime target_time;

  if (time.GetTime_s() == 0) {
    target_time = current_epoch_time_;
  } else {
    target_time = time;
  }

  double diff_s = target_time.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
  if (diff_s < 0.0 || diff_s > 1e6) return math::Vector<3>(0.0);

  const double kOrbitalPeriodCorrection_s = 24 * 60 * 60 * 1.003;  // See http://acc.igs.org/orbits/orbit-interp_gpssoln03.pdf
  return orbit_[0].CalcPositionWithTrigonometric(diff_s, math::tau / kOrbitalPeriodCorrection_s);
}

bool OrbitCalculationWithDefinitionFile::UpdateInterpolationInformation() {
  // OrbitDefinitionData orbit_definition_data = orbit_definition_data_;
  
    time_system::EpochTime orbit_definition_time = time_system::EpochTime(GetEpochData(reference_interpolation_id_));
    double time_diff_s = orbit_definition_time.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
    math::Vector<3> orbit_definition_position_eclipj2000_km;
    orbit_definition_position_eclipj2000_km(0) = orbit_definition_data_[reference_interpolation_id_].equ_x_eclipj2000_km;
    orbit_definition_position_eclipj2000_km(1) = orbit_definition_data_[reference_interpolation_id_].equ_y_eclipj2000_km;
    orbit_definition_position_eclipj2000_km(2) = orbit_definition_data_[reference_interpolation_id_].equ_z_eclipj2000_km;

    orbit_[0].PushAndPopData(time_diff_s, orbit_definition_position_eclipj2000_km);
  
  reference_interpolation_id_++;

  return true;
}

std::string OrbitCalculationWithDefinitionFile::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("satellite_position", "eclipj2000", "km", 3);


  return str_tmp;
}

std::string OrbitCalculationWithDefinitionFile::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(GetPosition_eclipj2000_km(), 16);

  return str_tmp;
}

OrbitCalculationWithDefinitionFile* InitOrbitCalculationWithDefinitionFile(const std::string file_name, const SimulationTime& simulation_time) {
  IniAccess ini_file(file_name);
  char section[] = "ORBIT_CALCULATION_WITH_DEFINITION_FILE";

  const bool is_calc_enable = ini_file.ReadEnable(section, INI_CALC_LABEL);
  const bool is_log_enable = ini_file.ReadEnable(section, INI_LOG_LABEL);

  OrbitCalculationWithDefinitionFile* orbit_calculation_with_definition_file = new OrbitCalculationWithDefinitionFile(is_calc_enable, is_log_enable);
  if (!orbit_calculation_with_definition_file->IsCalcEnabled()) {
    return orbit_calculation_with_definition_file;
  }

  const std::string orbit_definition_file_path = ini_file.ReadString(section, "orbit_definition_file_path");

  std::vector<OrbitDefinitionData> orbit_definition_data;
  orbit_calculation_with_definition_file->ReadOrbitDefinitionCsv(orbit_definition_file_path, orbit_definition_data);

  time_system::DateTime start_date_time((size_t)simulation_time.GetStartYear(), (size_t)simulation_time.GetStartMonth(),
                                        (size_t)simulation_time.GetStartDay(), (size_t)simulation_time.GetStartHour(),
                                        (size_t)simulation_time.GetStartMinute(), simulation_time.GetStartSecond());
  time_system::EpochTime start_epoch_time(start_date_time);
  orbit_calculation_with_definition_file->Initialize(orbit_definition_data, start_epoch_time, simulation_time);

  

  return orbit_calculation_with_definition_file;
}
