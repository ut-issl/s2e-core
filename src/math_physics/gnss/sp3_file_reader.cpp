/**
 * @file sp3_file_reader.cpp
 * @brief A class to read the SP3 (Extended Standard Product 3) format file and provide functions to access the data
 * @note Support version: SP3-d
 *       Ref: https://files.igs.org/pub/data/format/sp3d.pdf?_ga=2.115202830.823990648.1664976786-1792483726.1664976785
 */
#define _CRT_SECURE_NO_WARNINGS  // for sscanf

#include "sp3_file_reader.hpp"

#include <fstream>
#include <iostream>

namespace gnss {

Sp3FileReader::Sp3FileReader(const std::string file_name) { ReadFile(file_name); }

time_system::DateTime Sp3FileReader::GetEpochData(const size_t epoch_id) const {
  if (epoch_id > epoch_.size()) {
    time_system::DateTime zero;
    return zero;
  }
  return epoch_[epoch_id];
}

Sp3PositionClock Sp3FileReader::GetPositionClock(const size_t epoch_id, const size_t satellite_id) {
  Sp3PositionClock zero;
  if (epoch_id >= epoch_.size()) {
    return zero;
  }
  if (satellite_id >= position_clock_.size()) {
    return zero;
  }

  return position_clock_[satellite_id][epoch_id];
}

double Sp3FileReader::GetSatelliteClockOffset(const size_t epoch_id, const size_t satellite_id) {
  Sp3PositionClock position_clock = GetPositionClock(epoch_id, satellite_id);
  return position_clock.clock_us_;
}

math::Vector<3> Sp3FileReader::GetSatellitePosition_km(const size_t epoch_id, const size_t satellite_id) {
  Sp3PositionClock position_clock = GetPositionClock(epoch_id, satellite_id);
  return position_clock.position_km_;
}

bool Sp3FileReader::ReadFile(const std::string file_name) {
  // File open
  std::ifstream sp3_file(file_name);
  if (!sp3_file.is_open()) {
    std::cout << "[Warning] SP3 file not found: " << file_name << std::endl;
    return false;
  }

  // Header
  size_t line_number = ReadHeader(sp3_file);
  if (line_number == 0) return false;

  // Read epoch wise data
  for (size_t epoch_id = 0; epoch_id < header_.number_of_epoch_; epoch_id++) {
    std::string line;
    // Epoch information
    std::getline(sp3_file, line);
    if (line.find("* ") != 0) {
      std::cout << "[Warning] SP3 file Epoch line first character error: " << line << std::endl;
      return false;
    }
    size_t year, month, day, hour, minute;
    double second;
    sscanf(line.substr(3, 28).c_str(), "%zu %2zu %2zu %2zu %2zu %12lf", &year, &month, &day, &hour, &minute, &second);
    epoch_.push_back(time_system::DateTime(year, month, day, hour, minute, second));

    // Orbit and Clock information
    for (size_t satellite_id = 0; satellite_id < header_.number_of_satellites_; satellite_id++) {
      std::getline(sp3_file, line);
      // Position and Clock
      if (line.find("P") != 0) {
        std::cout << "[Warning] SP3 file position and clock data first character error: " << line << std::endl;
        return false;
      }
      Sp3PositionClock position_clock = DecodePositionClockData(line);
      position_clock_[satellite_id].push_back(position_clock);

      // [Optional] Position and Clock Correlation
      std::streampos previous_position = sp3_file.tellg();
      std::getline(sp3_file, line);
      if (line.find("EP") != 0) {
        sp3_file.seekg(previous_position);
      } else {
        Sp3PositionClockCorrelation position_clock_correlation = DecodePositionClockCorrelation(line);
        position_clock_correlation_[satellite_id].push_back(position_clock_correlation);
      }

      // Velocity and Clock rate
      if (header_.mode_ == Sp3Mode::kVelocity) {
        std::getline(sp3_file, line);
        // Position and Clock
        if (line.find("V") != 0) {
          std::cout << "[Warning] SP3 file position and clock data first character error: " << line << std::endl;
          return false;
        }
        Sp3VelocityClockRate velocity_clock_rate = DecodeVelocityClockRateData(line);
        velocity_clock_rate_[satellite_id].push_back(velocity_clock_rate);

        // [Optional] Velocity and Clock rate Correlation
        previous_position = sp3_file.tellg();
        std::getline(sp3_file, line);
        if (line.find("EV") != 0) {
          sp3_file.seekg(previous_position);
        } else {
          Sp3VelocityClockRateCorrelation velocity_clock_rate_correlation = DecodeVelocityClockRateCorrelation(line);
          velocity_clock_rate_correlation_[satellite_id].push_back(velocity_clock_rate_correlation);
        }
      }
    }
  }

  // Test
  time_system::DateTime test = epoch_[0];
  test = epoch_[1];
  std::vector<Sp3PositionClock> test_p = position_clock_[0];
  test_p = position_clock_[1];

  sp3_file.close();
  return true;
}

size_t Sp3FileReader::SearchNearestEpochId(const time_system::EpochTime time) {
  size_t nearest_epoch_id = 0;

  // Get header info
  const size_t num_epoch = header_.number_of_epoch_;
  const double interval_s = header_.epoch_interval_s_;

  // Check range
  time_system::EpochTime start_epoch(epoch_[0]);
  if (start_epoch > time) {
    nearest_epoch_id = 0;
  } else if ((time_system::EpochTime)(epoch_[num_epoch - 1]) < time) {
    nearest_epoch_id = num_epoch - 1;
  } else {  // Calc nearest point
    double diff_s = time.GetTimeWithFraction_s() - start_epoch.GetTimeWithFraction_s();
    nearest_epoch_id = (size_t)(diff_s / interval_s + 0.5);
  }

  return nearest_epoch_id;
}

size_t Sp3FileReader::ReadHeader(std::ifstream& sp3_file) {
  size_t line_number = 0;
  std::string line;

  // 1st line
  line_number++;
  std::getline(sp3_file, line);
  // Check SP3 version
  if (line.find("#d") != 0) {
    std::cout << "[Warning] SP3 file version is not supported: " << line << std::endl;
    std::cout << "We recommend to use SP3-d. " << std::endl;
  }
  // Read contents
  if (line[2] == 'P') {
    header_.mode_ = Sp3Mode::kPosition;
  } else if (line[3] == 'V') {
    header_.mode_ = Sp3Mode::kVelocity;
  } else {
    header_.mode_ = Sp3Mode::kOther;
    std::cout << "[Warning] SP3 file mode is undefined: " << line << std::endl;
  }
  size_t year, month, day, hour, minute;
  double second;
  sscanf(line.substr(3, 28).c_str(), "%zu %2zu %2zu %2zu %2zu %12lf", &year, &month, &day, &hour, &minute, &second);
  header_.start_epoch_ = time_system::DateTime(year, month, day, hour, minute, second);
  header_.number_of_epoch_ = std::stoi(line.substr(32, 7));
  header_.used_data_ = line.substr(40, 5);
  header_.coordinate_system_ = line.substr(46, 5);
  std::string orbit_type = line.substr(52, 3);
  if (orbit_type == "FIT") {
    header_.orbit_type_ = Sp3OrbitType::kFitted;
  } else if (orbit_type == "EXT") {
    header_.orbit_type_ = Sp3OrbitType::kExtrapolated;
  } else if (orbit_type == "BCT") {
    header_.orbit_type_ = Sp3OrbitType::kBroadcast;
  } else if (orbit_type == "HLM") {
    header_.orbit_type_ = Sp3OrbitType::kHelmert;
  } else {
    header_.orbit_type_ = Sp3OrbitType::kOther;
    std::cout << "[Warning] SP3 file orbit type is undefined: " << line << std::endl;
  }
  header_.agency_name_ = line.substr(56, 4);

  // 2nd line
  line_number++;
  std::getline(sp3_file, line);
  // Check first character
  if (line.find("##") != 0) {
    std::cout << "[Warning] SP3 file 2nd line first character error: " << line << std::endl;
    return 0;
  }
  // Read contents
  header_.start_gps_time_ = time_system::GpsTime(std::stoi(line.substr(3, 4)), std::stod(line.substr(8, 15)));
  header_.epoch_interval_s_ = std::stod(line.substr(24, 14));
  header_.start_time_mjday_ = std::stoi(line.substr(39, 5));
  header_.start_time_mjday_fractional_day_ = std::stod(line.substr(45, 15));

  // Satellite ID lines
  line_number++;
  std::getline(sp3_file, line);
  // Check first character
  if (line.find("+ ") != 0) {
    std::cout << "[Warning] SP3 file satellite ID line first character error: " << line << std::endl;
    return 0;
  }
  // Read contents
  header_.number_of_satellites_ = std::stoi(line.substr(3, 3));
  const size_t kMaxSatelliteNumberOneLine = 17;
  while (line.find("+ ") == 0) {
    for (size_t i = 0; i < kMaxSatelliteNumberOneLine; i++) {
      if (header_.satellite_ids_.size() >= header_.number_of_satellites_) {
        break;
      }
      header_.satellite_ids_.push_back(line.substr(9 + i * 3, 3));
    }
    line_number++;
    std::getline(sp3_file, line);
  }
  if (header_.satellite_ids_.size() != header_.number_of_satellites_) {
    std::cout << "[Warning] SP3 file number of satellite and size of satellite ID are incompatible." << std::endl;
    return 0;
  }

  // Accuracy lines
  // Check first character
  if (line.find("++") != 0) {
    std::cout << "[Warning] SP3 file accuracy line first character error: " << line << std::endl;
    return 0;
  }
  while (line.find("++") == 0) {
    for (size_t i = 0; i < kMaxSatelliteNumberOneLine; i++) {
      if (header_.satellite_accuracy_.size() >= header_.number_of_satellites_) {
        break;
      }
      header_.satellite_accuracy_.push_back((uint8_t)stoi(line.substr(9 + i * 3, 3)));
    }
    line_number++;
    std::getline(sp3_file, line);
  }
  if (header_.satellite_accuracy_.size() != header_.number_of_satellites_) {
    std::cout << "[Warning] SP3 file number of satellite and size of accuracy are incompatible." << std::endl;
    return 0;
  }

  // Additional character lines
  if (line.find("%c") != 0) {
    std::cout << "[Warning] SP3 file 1st additional character line first character error: " << line << std::endl;
    return 0;
  }
  header_.file_type_ = line.substr(3, 2);
  header_.time_system_ = line.substr(9, 3);
  line_number++;
  std::getline(sp3_file, line);
  if (line.find("%c") != 0) {
    std::cout << "[Warning] SP3 file 2nd additional character line first character error: " << line << std::endl;
    return 0;
  }

  // Additional float lines
  line_number++;
  std::getline(sp3_file, line);
  if (line.find("%f") != 0) {
    std::cout << "[Warning] SP3 file 1st additional float line first character error: " << line << std::endl;
    return 0;
  }
  header_.base_number_position_ = std::stod(line.substr(3, 10));
  header_.base_number_clock_ = std::stod(line.substr(14, 12));
  line_number++;
  std::getline(sp3_file, line);
  if (line.find("%f") != 0) {
    std::cout << "[Warning] SP3 file 2nd additional float line first character error: " << line << std::endl;
    return 0;
  }

  // Additional integer lines
  line_number++;
  std::getline(sp3_file, line);
  if (line.find("%i") != 0) {
    std::cout << "[Warning] SP3 file 1st additional integer line first character error: " << line << std::endl;
    return 0;
  }
  line_number++;
  std::getline(sp3_file, line);
  if (line.find("%i") != 0) {
    std::cout << "[Warning] SP3 file 2nd additional integer line first character error: " << line << std::endl;
    return 0;
  }

  // Comment lines
  std::streampos pos;
  do {
    line_number++;
    pos = sp3_file.tellg();
    std::getline(sp3_file, line);
  } while (line.find("/*") == 0);

  sp3_file.seekg(pos);
  return line_number - 1;
}

Sp3PositionClock Sp3FileReader::DecodePositionClockData(std::string line) {
  Sp3PositionClock position_clock;

  // Satellite ID
  position_clock.satellite_id_ = line.substr(1, 3);

  // Position and clock
  math::Vector<3> position_km;
  for (size_t axis = 0; axis < 3; axis++) {
    position_km[axis] = stod(line.substr(4 + axis * 14, 14));
  }
  position_clock.position_km_ = position_km;
  position_clock.clock_us_ = stod(line.substr(46, 14));

  // Standard deviations
  if (line.size() > 61) {
    math::Vector<3> position_standard_deviation;
    for (size_t axis = 0; axis < 3; axis++) {
      try {
        position_standard_deviation[axis] = stod(line.substr(61 + axis * 3, 2));
      } catch (const std::invalid_argument&) {
        position_standard_deviation[axis] = 0.0;
      }
    }
    position_clock.position_standard_deviation_ = position_standard_deviation;
    try {
      position_clock.clock_standard_deviation_ = stod(line.substr(70, 3));
    } catch (const std::invalid_argument&) {
      position_clock.clock_standard_deviation_ = 0.0;
    }
  }

  // Flags
  if (line.size() > 73) {
    if (line.substr(74, 1) == "E") {
      position_clock.clock_event_flag_ = true;
    }
    if (line.substr(75, 1) == "P") {
      position_clock.clock_prediction_flag_ = true;
    }
    if (line.substr(78, 1) == "M") {
      position_clock.maneuver_flag_ = true;
    }
    if (line.substr(79, 1) == "P") {
      position_clock.orbit_prediction_flag_ = true;
    }
  }

  // Register information
  return position_clock;
}

Sp3PositionClockCorrelation Sp3FileReader::DecodePositionClockCorrelation(std::string line) {
  Sp3PositionClockCorrelation correlation;

  // Satellite ID
  correlation.position_x_standard_deviation_mm_ = stoi(line.substr(4, 4));
  correlation.position_y_standard_deviation_mm_ = stoi(line.substr(9, 4));
  correlation.position_z_standard_deviation_mm_ = stoi(line.substr(14, 4));
  correlation.clock_standard_deviation_ps_ = stoi(line.substr(19, 7));
  correlation.x_y_correlation_ = stoi(line.substr(27, 8));
  correlation.x_z_correlation_ = stoi(line.substr(36, 8));
  correlation.x_clock_correlation_ = stoi(line.substr(45, 8));
  correlation.y_z_correlation_ = stoi(line.substr(54, 8));
  correlation.y_clock_correlation_ = stoi(line.substr(63, 8));
  correlation.z_clock_correlation_ = stoi(line.substr(72, 8));

  return correlation;
}

Sp3VelocityClockRate Sp3FileReader::DecodeVelocityClockRateData(std::string line) {
  Sp3VelocityClockRate velocity_clock_rate;

  // Satellite ID
  velocity_clock_rate.satellite_id_ = line.substr(1, 3);

  // Velocity and clock rate
  math::Vector<3> velocity_dm_s;
  for (size_t axis = 0; axis < 3; axis++) {
    velocity_dm_s[axis] = stod(line.substr(4 + axis * 14, 14));
  }
  velocity_clock_rate.velocity_dm_s_ = velocity_dm_s;
  velocity_clock_rate.clock_rate_ = stod(line.substr(46, 14));

  // Standard deviations
  if (line.size() > 60) {
    math::Vector<3> velocity_standard_deviation;
    for (size_t axis = 0; axis < 3; axis++) {
      velocity_standard_deviation[axis] = stod(line.substr(61 + axis * 2, 2));
    }
    velocity_clock_rate.velocity_standard_deviation_ = velocity_standard_deviation;
    velocity_clock_rate.clock_rate_standard_deviation_ = stod(line.substr(70, 3));
  }

  return velocity_clock_rate;
}

Sp3VelocityClockRateCorrelation Sp3FileReader::DecodeVelocityClockRateCorrelation(std::string line) {
  Sp3VelocityClockRateCorrelation correlation;

  // Satellite ID
  correlation.velocity_x_standard_deviation_ = stoi(line.substr(4, 4));
  correlation.velocity_y_standard_deviation_ = stoi(line.substr(9, 4));
  correlation.velocity_z_standard_deviation_ = stoi(line.substr(14, 4));
  correlation.clock_rate_standard_deviation_ = stoi(line.substr(19, 7));
  correlation.x_y_correlation_ = stoi(line.substr(27, 8));
  correlation.x_z_correlation_ = stoi(line.substr(36, 8));
  correlation.x_clock_correlation_ = stoi(line.substr(45, 8));
  correlation.y_z_correlation_ = stoi(line.substr(54, 8));
  correlation.y_clock_correlation_ = stoi(line.substr(63, 8));
  correlation.z_clock_correlation_ = stoi(line.substr(72, 8));

  return correlation;
}

}  // namespace gnss
