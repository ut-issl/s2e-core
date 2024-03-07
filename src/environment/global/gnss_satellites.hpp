/**
 * @file gnss_satellites.hpp
 * @brief Class to calculate GNSS satellite position and clock
 * @note TODO: Add GNSS satellite antenna information
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_GNSS_SATELLITES_HPP_
#define S2E_ENVIRONMENT_GLOBAL_GNSS_SATELLITES_HPP_

#include <math_physics/gnss/sp3_file_reader.hpp>
#include <math_physics/math/constants.hpp>
#include <math_physics/math/matrix_vector.hpp>
#include <math_physics/orbit/interpolation_orbit.hpp>
#include <math_physics/time_system/epoch_time.hpp>
#include <math_physics/time_system/gps_time.hpp>
#include <vector>

#include "earth_rotation.hpp"
#include "math_physics/gnss/gnss_satellite_number.hpp"
#include "math_physics/math/vector.hpp"
#include "logger/loggable.hpp"
#include "simulation_time.hpp"

/**
 * @class GnssSatellites
 * @brief Class to calculate GNSS satellite position and clock
 */
class GnssSatellites : public ILoggable {
 public:
  /**
   * @fn GnssSatellites
   * @brief Constructor
   * @param [in] earth_rotation: Earth rotation information
   * @param [in] is_calc_enabled: Flag to manage the GNSS satellite position/clock calculation
   * @param [in] is_log_enabled: Flag to generate the log of GNSS satellite position/clock calculation
   */
  GnssSatellites(const EarthRotation& earth_rotation, const bool is_calc_enabled = false, const bool is_log_enabled = false)
      : is_calc_enabled_(is_calc_enabled), earth_rotation_(earth_rotation) {
    if (!is_calc_enabled_) {
      is_log_enabled_ = false;
    } else {
      is_log_enabled_ = is_log_enabled;
    }
  }
  /**
   * @fn ~GnssSatellites
   * @brief Destructor
   */
  virtual ~GnssSatellites() {}

  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] sp3_files: List of SP3 files
   * @param [in] start_time: The simulation start time
   */
  void Initialize(const std::vector<Sp3FileReader>& sp3_files, const EpochTime start_time);

  /**
   * @fn IsCalcEnabled
   * @brief Return calculated enabled flag
   */
  inline bool IsCalcEnabled() const { return is_calc_enabled_; }

  /**
   * @fn GetNumberOfCalculatedSatellite
   * @brief Return number of calculated satellite
   */
  inline size_t GetNumberOfCalculatedSatellite() const { return number_of_calculated_gnss_satellites_; }

  /**
   * @fn Update
   * @brief Update both GNSS satellite information
   * @param [in] simulation_time: Simulation time information
   */
  void Update(const SimulationTime& simulation_time);

  inline libra::Vector<3> GetPosition_eci_m(const size_t gnss_satellite_id) const {
    // TODO: Add target time for earth rotation calculation
    return earth_rotation_.GetDcmJ2000ToEcef().Transpose() * GetPosition_ecef_m(gnss_satellite_id);
  }

  /**
   * @fn GetPosition_ecef_m
   * @brief Return GNSS satellite position at ECEF frame
   * @param [in] gnss_satellite_id: ID of GNSS satellite
   * @param [in] time: Target time to get the GNSS satellite. When the argument is not set, the last updated time is used for the calculation.
   * @return GNSS satellite position at ECEF frame at the time. Or return zero vector when the arguments are out of range.
   */
  libra::Vector<3> GetPosition_ecef_m(const size_t gnss_satellite_id, const EpochTime time = EpochTime(0, 0.0)) const;

  /**
   * @fn GetGetClock_s
   * @brief Return GNSS satellite clock offset
   * @param [in] gnss_satellite_id: ID of GNSS satellite
   * @param [in] time: Target time to get the GNSS satellite. When the argument is not set, the last updated time is used for the calculation.
   * @return GNSS satellite clock offset at the time. Or return zero when the arguments are out of range.
   */
  double GetClock_s(const size_t gnss_satellite_id, const EpochTime time = EpochTime(0, 0.0)) const;

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
  bool is_calc_enabled_ = false;  //!< Flag to manage the GNSS satellite position calculation

  std::vector<Sp3FileReader> sp3_files_;         //!< List of SP3 files
  size_t number_of_calculated_gnss_satellites_;  //!< Number of calculated GNSS satellites
  size_t sp3_file_id_;                           //!< Current SP3 file ID
  EpochTime reference_time_;                     //!< Reference start time of the SP3 handling
  size_t reference_interpolation_id_ = 0;        //!< Reference epoch ID of the interpolation
  EpochTime current_epoch_time_;                 //!< The last updated time

  std::vector<InterpolationOrbit> orbit_;    //!< GNSS satellite orbit with interpolation
  std::vector<libra::Interpolation> clock_;  //!< GNSS satellite clock offset with interpolation

  // References
  const EarthRotation& earth_rotation_;  //!< Earth rotation

  /**
   * @fn GetCurrentSp3File
   * @brief Get the SP3 file should be used at the time
   * @param [out] current_sp3_file: SP3 file information should be use.
   * @param [in] current_time: Target time
   * @return true means no error, false means the time argument is out of range
   */
  bool GetCurrentSp3File(Sp3FileReader& current_sp3_file, const EpochTime current_time);

  /**
   * @fn UpdateInterpolationInformation
   * @brief Update interpolation information by inserting new data
   * @return true: No error, false: SP3 file out of range error
   */
  bool UpdateInterpolationInformation();
};

/**
 * @fn InitGnssSatellites
 * @brief Initialize function for GnssSatellites class
 * @param [in] file_name: Path to the initialize file
 * @param [in] earth_rotation: Earth rotation information
 * @param [in] simulation_time: Simulation time information
 * @return Initialized GnssSatellite class
 */
GnssSatellites* InitGnssSatellites(const std::string file_name, const EarthRotation& earth_rotation, const SimulationTime& simulation_time);

#endif  // S2E_ENVIRONMENT_GLOBAL_GNSS_SATELLITES_HPP_
