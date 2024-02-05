/**
 * @file gnss_satellites.hpp
 * @brief Class to calculate GNSS satellite position and related states
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_GNSS_SATELLITES_HPP_
#define S2E_ENVIRONMENT_GLOBAL_GNSS_SATELLITES_HPP_

#include <library/gnss/sp3_file_reader.hpp>
#include <library/math/constants.hpp>
#include <library/math/matrix_vector.hpp>
#include <library/orbit/interpolation_orbit.hpp>
#include <library/time_system/epoch_time.hpp>
#include <library/time_system/gps_time.hpp>
#include <vector>

#include "library/gnss/gnss_satellite_number.hpp"
#include "library/logger/loggable.hpp"
#include "library/math/vector.hpp"
#include "simulation_time.hpp"

/**
 * @class GnssSatellites
 * @brief Class to calculate GNSS satellite position and related states
 */
class GnssSatellites : public ILoggable {
 public:
  /**
   * @fn GnssSatellites
   * @brief Constructor
   * @param [in] is_calc_enabled: Flag to manage the GNSS satellite position calculation
   */
  GnssSatellites(const bool is_calc_enabled = false, const bool is_log_enabled = false) : is_calc_enabled_(is_calc_enabled) {
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
   * @note Parameters are defined in GNSSSat_Info
   */
  void Initialize(const std::vector<Sp3FileReader>& sp3_files, const EpochTime start_time);
  /**
   * @fn IsCalcEnabled
   * @brief Return calculated enabled flag
   */
  inline bool IsCalcEnabled() const { return is_calc_enabled_; }

  /**
   * @fn SetUp
   * @brief Setup both GNSS satellite information
   * @param [in] simulation_time: Simulation time information
   */
  void SetUp(const SimulationTime* simulation_time);
  /**
   * @fn Update
   * @brief Update both GNSS satellite information
   * @param [in] simulation_time: Simulation time information
   */
  void Update(const SimulationTime& simulation_time);

  /**
   * @fn GetWhetherValid
   * @brief Return true the GNSS satellite information is available for both position and clock
   * @param [in] gnss_satellite_id: Index of GNSS satellite
   */
  inline bool GetWhetherValid(const size_t gnss_satellite_id) const { return true; }

  inline libra::Vector<3> GetPosition_eci_m(const size_t gnss_satellite_id) const { return libra::Vector<3>(0.0); }

  inline libra::Vector<3> GetPosition_ecef_m(const size_t gnss_satellite_id, const EpochTime time = EpochTime(0, 0.0)) const {
    EpochTime target_time;

    if (time.GetTime_s() == 0) {
      target_time = current_epoch_time_;
    } else {
      target_time = time;
    }

    double diff_s = target_time.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
    if (diff_s < 0.0 || diff_s > 1e6) return libra::Vector<3>(0.0);

    const double kOrbitalPeriodCorrection_s = 24 * 60 * 60 * 1.003;  // See http://acc.igs.org/orbits/orbit-interp_gpssoln03.pdf
    return orbit_[gnss_satellite_id].CalcPositionWithTrigonometric(diff_s, libra::tau / kOrbitalPeriodCorrection_s);
  }

  inline double GetClock_s(const size_t gnss_satellite_id, const EpochTime time = EpochTime(0, 0.0)) const {
    EpochTime target_time;

    if (time.GetTime_s() == 0) {
      target_time = current_epoch_time_;
    } else {
      target_time = time;
    }

    double diff_s = target_time.GetTimeWithFraction_s() - reference_time_.GetTimeWithFraction_s();
    if (diff_s < 0.0 || diff_s > 1e6) return 0.0;

    return clock_[gnss_satellite_id].CalcPolynomial(diff_s) * 1e-6;
  }

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
  bool is_calc_enabled_ = true;  //!< Flag to manage the GNSS satellite position calculation

  std::vector<Sp3FileReader> sp3_files_;   //!< List of SP3 files
  size_t sp3_file_id_;                     //!< Current SP3 file ID
  EpochTime reference_time_;               //!< Reference start time of the SP3 handling
  size_t reference_interpolation_id_ = 0;  //!< Reference epoch ID of the interpolation

  std::vector<InterpolationOrbit> orbit_;    //!< GNSS satellite orbit with interpolation
  std::vector<libra::Interpolation> clock_;  //!< GNSS satellite clock offset with interpolation

  EpochTime current_epoch_time_;

  // Check
  bool GetCurrentSp3File(Sp3FileReader& current_sp3_file, const EpochTime current_time);
};

/**
 *@fn InitGnssSatellites
 *@brief Initialize function for GnssSatellites class
 *@param [in] file_name: Path to the initialize file
 */
GnssSatellites* InitGnssSatellites(const std::string file_name, const SimulationTime& simulation_time);

#endif  // S2E_ENVIRONMENT_GLOBAL_GNSS_SATELLITES_HPP_
