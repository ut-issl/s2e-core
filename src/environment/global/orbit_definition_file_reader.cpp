/**
 * @file orbit_definition_file_reader.cpp
 * @brief A class to read the orbit definition file and provide functions to access the data
 */

#include "orbit_definition_file_reader.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

bool OrbitDefinitionFileReader::ReadOrbitDefinitionCsv(const std::string& file_name, char delimiter) {
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

    OrbitDefinitionData data;

    // Read time (UTC)
    std::getline(streamline, cell, delimiter);
    data.time_utc = cell;

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

    orbit_definition_data_.push_back(data);
  }
  return true;
}