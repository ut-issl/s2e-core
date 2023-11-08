/**
 * @file antex_file_reader.hpp
 * @brief Read ANTEX format file
 */
#define _CRT_SECURE_NO_WARNINGS  // for sscanf

#include "antex_file_reader.hpp"

#include <fstream>
#include <iostream>
#include <library/gnss/gnss_satellite_number.hpp>

#define ANTEX_LINE_TYPE_POSITION (60)

AntexGridDefinition::AntexGridDefinition(const double zenith_start_angle_deg, const double zenith_end_angle_deg, const double zenith_step_angle_deg,
                                         const double azimuth_step_angle_deg)
    : zenith_start_angle_deg_(zenith_start_angle_deg),
      zenith_end_angle_deg_(zenith_end_angle_deg),
      zenith_step_angle_deg_(zenith_step_angle_deg),
      azimuth_step_angle_deg_(azimuth_step_angle_deg) {
  if (zenith_step_angle_deg_ <= 0.0) {
    number_of_zenith_grid_ = 0;
  } else {
    number_of_zenith_grid_ = size_t((zenith_end_angle_deg_ - zenith_start_angle_deg_) / zenith_step_angle_deg_) + 1;
  }

  if (azimuth_step_angle_deg_ <= 0.0) {
    number_of_azimuth_grid_ = 0;
  } else {
    number_of_azimuth_grid_ = size_t(360.0 / azimuth_step_angle_deg_) + 1;
  }
}

size_t AntexGridDefinition::CalcClosestZenithIndex(const double zenith_angle_deg) {
  if (zenith_angle_deg <= zenith_start_angle_deg_) {
    return 0;
  } else if (zenith_angle_deg >= zenith_end_angle_deg_) {
    return number_of_zenith_grid_ - 1;
  } else {
    double diff_angle_deg = zenith_angle_deg - zenith_start_angle_deg_;
    size_t diff_index = size_t(diff_angle_deg / zenith_step_angle_deg_ + 0.5);
    return diff_index;
  }
}

size_t AntexGridDefinition::CalcClosestAzimuthIndex(const double azimuth_angle_deg) {
  if (azimuth_angle_deg <= 0.0) {
    return 0;
  } else if (azimuth_angle_deg >= 360.0) {
    return number_of_azimuth_grid_ - 1;
  } else {
    size_t diff_index = size_t(azimuth_angle_deg / azimuth_step_angle_deg_ + 0.5);
    return diff_index;
  }
}

bool AntexFileReader::ReadFile(const std::string file_name) {
  // File open
  std::ifstream antex_file(file_name);
  if (!antex_file.is_open()) {
    std::cout << "[Warning] Antex file not found: " << file_name << std::endl;
    return false;
  }

  while (antex_file.peek() != EOF) {
    std::string line;
    std::getline(antex_file, line);
    // Skip short line
    if (line.size() < ANTEX_LINE_TYPE_POSITION) {
      continue;
    }
    // Skip comments
    if (line.find("COMMENT") == ANTEX_LINE_TYPE_POSITION) {
      continue;
    }
    // Start antenna
    if (line.find("START OF ANTENNA") == ANTEX_LINE_TYPE_POSITION) {
      ReadAntexData(antex_file);
    }
  }

  // Read epoch wise data
  antex_file.close();
  return true;
}

void AntexFileReader::ReadAntexData(std::ifstream& antex_file) {
  AntexSatelliteData antex_data;

  std::string line;
  while (1) {
    std::getline(antex_file, line);

    // Type and Serial No.
    if (line.find("TYPE / SERIAL NO") == ANTEX_LINE_TYPE_POSITION) {
      std::string antenna_type = line.substr(0, 20);
      std::string serial_number = line.substr(20, 20);
      size_t satellite_index = ConvertSatelliteNumberToIndex(serial_number);

      if (satellite_index == UINT32_MAX) {
        // receiver
        // TODO: implement
      } else {
        // GNSS satellite
        AntexSatelliteData antex_satellite_data;
        antex_satellite_data = ReadAntexSatelliteData(antex_file);
        antex_satellite_data.SetAntennaType(antenna_type);
        antex_satellite_data.SetSerialNumber(serial_number);
        antex_satellite_data_[satellite_index].push_back(antex_satellite_data);
      }
      break;
    }
  }
}

AntexSatelliteData AntexFileReader::ReadAntexSatelliteData(std::ifstream& antex_file) {
  AntexSatelliteData antex_data;
  AntexGridDefinition grid;
  std::vector<AntexPhaseCenterData> phase_center_data_list;

  std::string line;
  while (1) {
    std::getline(antex_file, line);

    // End antenna data
    if (line.find("END OF ANTENNA") == ANTEX_LINE_TYPE_POSITION) {
      antex_data.SetPhaseCenterData(phase_center_data_list);
      break;
    }
    // Grid angle definition
    if (line.find("ZEN1 / ZEN2 / DZEN") == ANTEX_LINE_TYPE_POSITION) {
      double start, end, step;
      sscanf(line.substr(0, 60).c_str(), "%lf %lf %lf", &start, &end, &step);
      grid = AntexGridDefinition(start, end, step);
    }
    // Number of frequency
    if (line.find("# OF FREQUENCIES") == ANTEX_LINE_TYPE_POSITION) {
      antex_data.SetNumberOfFrequency(std::stoi(line.substr(0, 20)));
    }
    // Valid from
    if (line.find("VALID FROM") == ANTEX_LINE_TYPE_POSITION) {
      antex_data.SetValidStartTime(ReadDateTime(line.substr(0, 59)));
    }
    // Valid until
    if (line.find("VALID UNTIL") == ANTEX_LINE_TYPE_POSITION) {
      antex_data.SetValidEndTime(ReadDateTime(line.substr(0, 59)));
    }
    // Frequency
    if (line.find("START OF FREQUENCY") == ANTEX_LINE_TYPE_POSITION) {
      AntexPhaseCenterData phase_center_data = ReadPhaseCenterData(antex_file, grid);
      phase_center_data_list.push_back(phase_center_data);
    }
  }

  return antex_data;
}

AntexPhaseCenterData AntexFileReader::ReadPhaseCenterData(std::ifstream& antex_file, const AntexGridDefinition grid_information) {
  AntexPhaseCenterData phase_center_data;
  std::string line;
  while (1) {
    std::getline(antex_file, line);

    // End antenna data
    if (line.find("END OF FREQUENCY") == ANTEX_LINE_TYPE_POSITION) {
      phase_center_data.SetFrequencyName(line.substr(3, 3));
      phase_center_data.SetGridInformation(grid_information);
      break;
    }
    // Phase center offset
    if (line.find("NORTH / EAST / UP") == ANTEX_LINE_TYPE_POSITION) {
      libra::Vector<3> offset{0.0};
      sscanf(line.c_str(), "%lf %lf %lf", &offset[0], &offset[1], &offset[2]);
      phase_center_data.SetPhaseCenterOffset_mm(offset);
    }
    // Phase center variation
    if (line.find("NOAZI") != std::string::npos) {
      std::vector<double> phase_center_variation;
      for (size_t i = 0; i < grid_information.GetNumberOfZenithGrid(); i++) {
        double parameter = std::stod(line.substr(8 + i * 8, 8));
        phase_center_variation.push_back(parameter);
      }
      std::vector<std::vector<double>> phase_center_variation_matrix;
      phase_center_variation_matrix.push_back(phase_center_variation);
      phase_center_data.SetPhaseCenterVariationMatrix_mm(phase_center_variation_matrix);
    }
    // TODO: implement DAZI
  }

  return phase_center_data;
}

DateTime AntexFileReader::ReadDateTime(std::string line) {
  size_t year, month, day, hour, minute;
  double second;
  sscanf(line.c_str(), "%zu %2zu %2zu %2zu %2zu %10lf", &year, &month, &day, &hour, &minute, &second);
  return DateTime(year, month, day, hour, minute, second);
}
