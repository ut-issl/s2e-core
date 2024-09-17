/**
 * @file orbit_definition_file_reader.hpp
 * @brief A class to read the orbit definition file and provide functions to access the data
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_ORBIT_DEFINITION_FILE_READER_HPP_
#define S2E_ENVIRONMENT_GLOBAL_ORBIT_DEFINITION_FILE_READER_HPP_

#include <string>
#include <vector>

/**
 *@struct OrbitDefinitionData
 *@brief Orbit definition data
 */
struct OrbitDefinitionData {
  std::string time_utc;         //!< time (UTC)
  double time_et_s;             //!< Ephemeris time [s]
  double equ_x_eclipj2000_km;   //!< EQU X(ECLIPJ2000) [km]
  double equ_y_eclipj2000_km;   //!< EQU Y(ECLIPJ2000) [km]
  double equ_z_eclipj2000_km;   //!< EQU Z(ECLIPJ2000) [km]
  double vx_eclipj2000_km_s;    //!< VX [km/s]
  double vy_eclipj2000_km_s;    //!< VY [km/s]
  double vz_eclipj2000_km_s;    //!< VZ [km/s]
  double m_kg;                  //!< Mass [kg]
  double fx_kN;                 //!< FX [kN]
  double fy_kN;                 //!< FY [kN]
  double fz_kN;                 //!< FZ [kN]
  bool dv_node;                 //!< DV NODE
  double dv_x_eclipj2000_km_s;  //!< DV_X [km/s]
  double dv_y_eclipj2000_km_s;  //!< DV_Y [km/s]
  double dv_z_eclipj2000_km_s;  //!< DV_Z [km/s]
  double sun_x_eclipj2000_km;   //!< SUN X (ECLIPJ2000) [km]
  double sun_y_eclipj2000_km;   //!< SUN Y (ECLIPJ2000) [km]
  double sun_z_eclipj2000_km;   //!< SUN Z (ECLIPJ2000) [km]
  double moon_x_eclipj2000_km;  //!< MOON X (ECLIPJ2000) [km]
  double moon_y_eclipj2000_km;  //!< MOON Y (ECLIPJ2000) [km]
  double moon_z_eclipj2000_km;  //!< MOON Z (ECLIPJ2000) [km]
};

/**
 * @class OrbitDefinitionFileReader
 * @brief
 */
class OrbitDefinitionFileReader {
 public:
  /**
   * @fn ReadOrbitDefinitionCsv
   * @brief Read orbit definition CSV file.
   * @param file_name Path to orbit definition CSV file.
   * @param delimiter Delimiter for the orbit definition CSV file (default: ',').
   */
  bool ReadOrbitDefinitionCsv(const std::string& file_name, char delimiter = ',');
};

#endif  // S2E_ENVIRONMENT_GLOBAL_ORBIT_DEFINITION_FILE_READER_HPP_