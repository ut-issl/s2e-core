/**
 * @file time_series_file_orbit_propagation.hpp
 * @brief Class to calculate satellite orbit using interpolation with orbit time series input
 */

#ifndef S2E_DYNAMICS_ORBIT_TIME_SERIES_FILE_ORBIT_PROPAGATION_HPP_
#define S2E_DYNAMICS_ORBIT_TIME_SERIES_FILE_ORBIT_PROPAGATION_HPP_

#include <math_physics/orbit/interpolation_orbit.hpp>
#include <math_physics/time_system/epoch_time.hpp>
#include <string>
#include <vector>

#include "orbit.hpp"

/**
 * @class TimeSeriesFileOrbitPropagation
 * @brief Class to calculate satellite orbit using interpolation with orbit time series input
 */
class TimeSeriesFileOrbitPropagation : public Orbit {
 public:
  /**
   *@fn TimeSeriesFileOrbitPropagation
   *@brief Constructor
   * @param [in] celestial_information: Celestial information
   * @param [in] time_series_file_path: Path to the time series file
   * @param [in] number_of_interpolation: Number of interpolation
   * @param [in] interpolation_method: Interpolation method
   * @param [in] orbital_period_correction_s: Orbital period correction [s]
   * @param [in] current_time_jd: Current Julian day [day]
   */
  TimeSeriesFileOrbitPropagation(const CelestialInformation* celestial_information, std::string time_series_file_path, int number_of_interpolation,
                                 int interpolation_method, double orbital_period_correction_s, const double current_time_jd);

  /**
   *@fn ~TimeSeriesFileOrbitPropagation
   *@brief Destructor
   */
  virtual ~TimeSeriesFileOrbitPropagation() {}

  /**
   * @fn CalcEpochData
   * @brief Return epoch data for a specific epoch ID.
   * @param epoch_id The epoch ID of the orbit definition data.
   */
  time_system::DateTime CalcEpochData(const size_t epoch_id) const;

  /**
   * @fn SearchNearestEpochId
   * @brief Search the nearest epoch ID from the orbit definition data.
   * @param current_time_jd: Current Julian day [day]
   */
  size_t SearchNearestEpochId(const double current_time_jd);

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] end_time_s: End time of simulation [sec]
   * @param [in] current_time_jd: Current Julian day [day]
   */
  virtual void Propagate(const double end_time_s, const double current_time_jd);

 private:
  bool is_time_range_warning_displayed_ = false;          //!< Flag for time range warning
  bool is_interpolation_method_error_displayed_ = false;  //!< Flag for interpolation method error

  int number_of_interpolation_;         //!< Number of interpolation
  int interpolation_method_;            //!< Interpolation method
  double orbital_period_correction_s_;  //!< Orbital period correction [s]

  std::vector<time_system::DateTime> epoch_;           //!< Epoch data list
  std::vector<std::vector<double>> time_series_data_;  //!< List of orbit definition data
  time_system::EpochTime current_epoch_time_;          //!< The last updated time
  time_system::EpochTime reference_time_;              //!< Reference start time of the orbit definition data handling
  size_t reference_interpolation_id_ = 0;              //!< Reference epoch ID of the interpolation

  std::vector<orbit::InterpolationOrbit> orbit_position_i_m_;    //!< Position with interpolation
  std::vector<orbit::InterpolationOrbit> orbit_velocity_i_m_s_;  //!< Velocity with interpolation

  /**
   * @fn UpdateInterpolationInformation
   * @brief Update interpolation information by inserting new data
   * @return true: No error, false: Orbit definition file out of range error
   */
  bool UpdateInterpolationInformation();
};

#endif  // S2E_DYNAMICS_ORBIT_TIME_SERIES_FILE_ORBIT_PROPAGATION_HPP_
