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

  size_t line_number = ReadHeader(sp3_file);
  if (line_number == 0) return false;

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

  // 3rd - 11th line
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

  // 12th - 20th line
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
  

  return line_number;
}
