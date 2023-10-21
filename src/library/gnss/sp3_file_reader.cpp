/**
 * @file sp3_file_reader.cpp
 * @brief A class to read the SP3 (Extended Standard Product 3) format file and provide functions to access the data
 * @note Support version: SP3-d
 *       Ref: https://files.igs.org/pub/data/format/sp3d.pdf?_ga=2.115202830.823990648.1664976786-1792483726.1664976785
 */

#include "sp3_file_reader.hpp"

#include <fstream>
#include <iostream>

Sp3FileReader::Sp3FileReader(const std::string file_name) { ReadFile(file_name); }

bool Sp3FileReader::ReadFile(const std::string file_name) {
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
    epoch_.push_back(DateTime(year, month, day, hour, minute, second));

    // Orbit and Clock information
    for (size_t satellite_id = 0; satellite_id < header_.number_of_satellites_; satellite_id++) {
      std::getline(sp3_file, line);
      if (line.find("P") == 0) {  // Position and Clock
        Sp3PositionClock position_clock;
        position_clock.satellite_id_ = line.substr(1, 3);
        // Position and clock
        libra::Vector<3> position_km;
        for (size_t axis = 0; axis < 3; axis++) {
          position_km[axis] = stod(line.substr(4 + axis * 14, 14));
        }
        position_clock.position_km_ = position_km;
        position_clock.clock_us_ = stod(line.substr(46, 14));

        // Standard deviations
        if (line.size() > 60) {
          libra::Vector<3> position_standard_deviation;
          for (size_t axis = 0; axis < 3; axis++) {
            position_standard_deviation[axis] = stod(line.substr(61 + axis * 2, 2));
          }
          position_clock.position_standard_deviation_ = position_standard_deviation;
          position_clock.clock_standard_deviation_ = stod(line.substr(70, 3));
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
        position_clock_.push_back(position_clock);
      } else if (line.find("EP") == 0) {  // Position and Clock correlation
      } else if (line.find("V") == 0) {   // Velocity and Clock rate
      } else if (line.find("EV") == 0) {  // Velocity and Clock rate correlation
      } else {
        std::cout << "[Warning] SP3 file Orbit data line first character error: " << line << std::endl;
        return false;
      }
    }
  }

  sp3_file.close();
  return true;
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
    return 0;
  }
  // Read contents
  if (line[2] == 'P') {
    header_.mode_ = Sp3Mode::kPosition;
  } else if (line[3] == 'V') {
    header_.mode_ = Sp3Mode::kVelocity;
  } else {
    std::cout << "[Warning] SP3 file mode is undefined: " << line << std::endl;
    return 0;
  }
  size_t year, month, day, hour, minute;
  double second;
  sscanf(line.substr(3, 28).c_str(), "%zu %2zu %2zu %2zu %2zu %12lf", &year, &month, &day, &hour, &minute, &second);
  header_.start_epoch_ = DateTime(year, month, day, hour, minute, second);
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
    std::cout << "[Warning] SP3 file orbit type is undefined: " << line << std::endl;
    return 0;
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
  header_.start_gps_time_ = GpsTime(std::stoi(line.substr(3, 4)), std::stod(line.substr(8, 15)));
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
      header_.satellite_ids_.push_back(line.substr(9 + i * 3, 3));
      if (header_.satellite_ids_.size() >= header_.number_of_satellites_) {
        break;
      }
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
      header_.satellite_accuracy_.push_back((uint8_t)stoi(line.substr(9 + i * 3, 3)));
      if (header_.satellite_accuracy_.size() >= header_.number_of_satellites_) {
        break;
      }
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
