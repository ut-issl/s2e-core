/**
 * @file atmosphere.hpp
 * @brief Class to calculate earth's atmospheric density
 */
#pragma once

#ifndef S2E_ENVIRONMENT_LOCAL_ATMOSPHERE_HPP_
#define S2E_ENVIRONMENT_LOCAL_ATMOSPHERE_HPP_

#include <string>
#include <vector>

#include "library/external/nrlmsise00/wrapper_nrlmsise00.hpp"
#include "library/geodesy/geodetic_position.hpp"
#include "library/logger/loggable.hpp"
#include "library/math/vector.hpp"

/**
 * @class Atmosphere
 * @brief Class to calculate earth's atmospheric density
 */
class Atmosphere : public ILoggable {
 public:
  bool IsCalcEnabled = true;  //!< Calculation enable flag

  /**
   * @fn Atmosphere
   * @brief Constructor
   * @param [in] model: Atmospheric density model name
   * @param [in] initialize_file_name: Path and name of initialize file
   * @param [in] gauss_standard_deviation_rate: Standard deviation of density noise (defined as percentage)
   * @param [in] is_manual_param: Flag to use manual parameters
   * @param [in] manual_f107: Manual value of daily F10.7
   * @param [in] manual_f107a: Manual value of averaged F10.7 (3-month averaged value)
   * @param [in] manual_ap: Manual value of ap value
   */
  Atmosphere(const std::string model, const std::string initialize_file_name, const double gauss_standard_deviation_rate, const bool is_manual_param,
             const double manual_f107, const double manual_f107a, const double manual_ap);
  /**
   * @fn ~Atmosphere
   * @brief Destructor
   */
  virtual ~Atmosphere() {}
  /**
   * @fn CalcAirDensity
   * @brief Calculate atmospheric density
   * @param [in] decimal_year: Decimal year of simulation start [year]
   * @param [in] end_time_s: End time of simulation [sec]
   * @param [in] position: Position of target point to calculate the air density
   * @return Atmospheric density [kg/m^3]
   */
  double CalcAirDensity_kg_m3(const double decimal_year, const double end_time_s, const GeodeticPosition position);
  /**
   * @fn GetAirDensity
   * @brief Return Atmospheric density [kg/m^3]
   */
  inline double GetAirDensity_kg_m3() const { return air_density_kg_m3_; }

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

 private:
  std::string model_;                                //!< Atmospheric density model name
  std::string initialize_file_name_;                 //!< Path and name of initialize file
  double air_density_kg_m3_;                         //!< Atmospheric density [kg/m^3]
  double gauss_standard_deviation_rate_;             //!< Standard deviation of density noise (defined as percentage)
  std::vector<nrlmsise_table> space_weather_table_;  //!< Space weather table
  bool is_space_weather_table_imported_;             //!< Flag of the space weather table is imported or not
  bool is_manual_param_used_;                        //!< Flag to use manual parameters

  // Reference of the following setting parameters https://www.swpc.noaa.gov/phenomena/f107-cm-radio-emissions
  double manual_daily_f107_;    //!< Manual daily f10.7 value
  double manual_average_f107_;  //!< Manual 3-month averaged f10.7 value
  double manual_ap_;            //!< Manual ap value Ref: http://wdc.kugi.kyoto-u.ac.jp/kp/kpexp-j.html

  // TODO: Add random walk noise
  //  double rw_stepwidth_;
  //  double rw_stddev_;
  //  double rw_limit_;

  /**
   * @fn CalcStandard
   * @brief Calculate atmospheric density with simplest method
   * @param [in] altitude_m: Altitude of spacecraft [m]
   * @return Atmospheric density [kg/m^3]
   */
  double CalcStandard(const double altitude_m);
  /**
   * @fn GetSpaceWeatherTable
   * @param [in] decimal_year: Decimal year of simulation start [year]
   * @param [in] end_time_s: End time of simulation [sec]
   * @return Size of table
   */
  int GetSpaceWeatherTable(const double decimal_year, const double end_time_s);

  /**
   * @fn AddNoise
   * @brief Add atmospheric density noise
   * @param [in] rho_kg_m3: True atmospheric density [kg/m^3]
   * @return Atmospheric density with noise [kg/m^3]
   */
  double AddNoise(const double rho_kg_m3);
};

#endif  // S2E_ENVIRONMENT_LOCAL_ATMOSPHERE_HPP_
