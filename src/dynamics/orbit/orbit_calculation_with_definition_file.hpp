/**
 * @file orbit_calculation_with_defnition_file.hpp
 * @brief Class to calculate satellite orbit using interpolation with orbit time series input
 */

#ifndef S2E_DYNAMICS_ORBIT_ORBIT_CALCULATION_WITH_DEFINITION_FILE_HPP_
#define S2E_DYNAMICS_ORBIT_ORBIT_CALCULATION_WITH_DEFINITION_FILE_HPP_

#include <string>
#include <vector>


// #include <math_physics/gnss/sp3_file_reader.hpp>
// #include <math_physics/math/constants.hpp>
// #include <math_physics/math/matrix_vector.hpp>
#include <math_physics/orbit/interpolation_orbit.hpp>
#include <math_physics/time_system/epoch_time.hpp>
// #include <math_physics/time_system/gps_time.hpp>
// #include <vector>

// #include "earth_rotation.hpp"
// #include "logger/loggable.hpp"
// #include "math_physics/gnss/gnss_satellite_number.hpp"
// #include "math_physics/math/vector.hpp"
#include "simulation_time.hpp"

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
 * @class OrbitCalculationWithDefinitionFile
 * @brief Class to calculate satellite orbit using interpolation with orbit time series input
 */
class OrbitCalculationWithDefinitionFile : public ILoggable  {
 public:
  /**
   *@fn OrbitCalculationWithDefinitionFile
   *@brief Constructor
   * @param [in] is_calc_enabled: Flag to manage the orbit calculation
   * @param [in] is_log_enabled: Flag to generate the log of orbit calculation
   */
  OrbitCalculationWithDefinitionFile(const bool is_calc_enabled = false, const bool is_log_enabled = false)
      : is_calc_enabled_(is_calc_enabled) {
    if (!is_calc_enabled_) {
      is_log_enabled_ = false;
    } else {
      is_log_enabled_ = is_log_enabled;
    }
  }

  /**
   *@fn ~OrbitCalculationWithDefinitionFile
   *@brief Destructor
   */  
  virtual ~OrbitCalculationWithDefinitionFile() {}

  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] orbit_definition_data: orbit definition data
   * @param [in] start_time: The simulation start time
   */
  void Initialize(const std::vector<OrbitDefinitionData>& orbit_definition_data, const time_system::EpochTime start_time, const SimulationTime& simulation_time);

  /**
   * @fn IsCalcEnabled
   * @brief Return calculated enabled flag
   */
  inline bool IsCalcEnabled() const { return is_calc_enabled_; }

  /**
   * @fn ReadOrbitDefinitionCsv
   * @brief Read orbit definition CSV file.
   * @param file_name Path to orbit definition CSV file.
   * @param orbit_definition_data List of orbit definition data.
   * @param delimiter Delimiter for the orbit definition CSV file (default: ',').
   */
  bool ReadOrbitDefinitionCsv(const std::string& file_name, std::vector<OrbitDefinitionData>& orbit_definition_data, char delimiter = ',');

  /**
   * @fn GetOrbitDefinitionDataSize
   * @brief Return read orbit definition data size.
   */
  size_t GetOrbitDefinitionDataSize() const { return orbit_definition_data_.size(); }

  /**
   * @fn GetOrbitDefinitionData
   * @brief Return orbit definition data for a specific index.
   * @param index The index of the orbit definition data to.
   */
  OrbitDefinitionData GetOrbitDefinitionData(size_t index) const { return orbit_definition_data_[index]; }

  /**
   * @fn GetEpochData
   * @brief Return epoch data for a specific epoch ID.
   * @param epoch_id The epoch ID of the orbit definition data.
   */
  time_system::DateTime GetEpochData(const size_t epoch_id) const;

  /**
   * @fn SearchNearestEpochId
   * @brief Search the nearest epoch ID from the orbit definition data.
   * @param simulation_time The simulation time information.
   */
  size_t SearchNearestEpochId(const SimulationTime& simulation_time);


  /**
   * @fn Update
   * @brief Updatesatellite information
   * @param [in] simulation_time: Simulation time information
   */ 
  void Update(const SimulationTime& simulation_time);

  /**
   * @fn GetPosition_eclipj2000_km
   * @brief Return satellite position at ECLIPJ2000 frame
   * @param [in] time: Target time to get the satellite. When the argument is not set, the last updated time is used for the calculation.
   * @return Satellite position at ECLIPJ2000 frame at the time. Or return zero vector when the arguments are out of range.
   */
  math::Vector<3> GetPosition_eclipj2000_km(const time_system::EpochTime time = time_system::EpochTime(0, 0.0)) const;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  std::string GetLogValue() const override;

 private:
  bool is_calc_enabled_ = false;  //!< Flag to manage the orbit calculation

  std::vector<time_system::DateTime> epoch_;  //!< Epoch data list

  std::vector<OrbitDefinitionData> orbit_definition_data_;  //!< List of orbit definition data
  time_system::EpochTime current_epoch_time_;    //!< The last updated time
  time_system::EpochTime reference_time_;        //!< Reference start time of the orbit definition data handling
  size_t reference_interpolation_id_ = 0;        //!< Reference epoch ID of the interpolation
  time_system::EpochTime current_epoch_time_;    //!< The last updated time
  
  std::vector<orbit::InterpolationOrbit> orbit_;  //!< Satellite orbit with interpolation

  /**
   * @fn UpdateInterpolationInformation
   * @brief Update interpolation information by inserting new data
   * @return true: No error, false: Orbit definition file out of range error
   */
  bool UpdateInterpolationInformation();
};

/**
 * @fn InitOrbitCalculationWithDefinitionFile
 * @brief Initialize function for OrbitCalculationWithDefinitionFile class
 * @param [in] file_name: Path to the initialize file
 * @param [in] simulation_time: Simulation time information
 * @return Initialized OrbitCalculationWithDefinitionFile class
 */
OrbitCalculationWithDefinitionFile* InitOrbitCalculationWithDefinitionFile(const std::string file_name, const SimulationTime& simulation_time);
#endif  // S2E_DYNAMICS_ORBIT_ORBIT_CALCULATION_WITH_DEFINITION_FILE_HPP_