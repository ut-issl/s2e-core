/**
 * @file atmosphere.hpp
 * @brief Class to calculate earth's atmospheric density
 */
#pragma once

#ifndef S2E_ENVIRONMENT_LOCAL_ATMOSPHERE_H_
#define S2E_ENVIRONMENT_LOCAL_ATMOSPHERE_H_

#include <library/nrlmsise00/Wrapper_nrlmsise00.h>

#include <library/math/Quaternion.hpp>
#include <library/math/Vector.hpp>
#include <interface/log_output/loggable.hpp>
#include <string>
#include <vector>

using libra::Quaternion;
using libra::Vector;

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
   * @param [in] fname: Path and name of initialize file
   * @param [in] gauss_stddev: Standard deviation of density noise (defined as percentage)
   * @param [in] is_manual_param: Flag to use manual parameters
   * @param [in] manual_f107: Manual value of daily F10.7
   * @param [in] manual_f107a: Manual value of averaged F10.7 (3-month averaged value)
   * @param [in] manual_ap: Manual value of ap value
   */
  Atmosphere(std::string model, std::string fname, double gauss_stddev, bool is_manual_param, double manual_f107, double manual_f107a,
             double manual_ap);
  /**
   * @fn ~Atmosphere
   * @brief Destructor
   */
  virtual ~Atmosphere() {}
  /**
   * @fn CalcAirDensity
   * @brief Calculate atmospheric density
   * @param [in] decyear: Decimal year of simulation start [year]
   * @param [in] endsec: End time of simulation [sec]
   * @param [in] lat_lon_alt: Latitude[rad], longitude[rad], and altitude[m]
   * @return Atmospheric density [kg/m^3]
   */
  double CalcAirDensity(double decyear, double endsec, Vector<3> lat_lon_alt);
  /**
   * @fn GetAirDensity
   * @brief Return Atmospheric density [kg/m^3]
   */
  double GetAirDensity() const;

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
  std::string model_;                  //!< Atmospheric density model name
  std::string fname_;                  //!< Path and name of initialize file
  double air_density_;                 //!< Atmospheric density [kg/m^3]
  double gauss_stddev_;                //!< Standard deviation of density noise (defined as percentage)
  std::vector<nrlmsise_table> table_;  //!< Space weather table
  bool is_table_imported_;             //!< Flag of the space weather table is imported or not
  bool is_manual_param_used_;          //!< Flag to use manual parameters

  // Reference of the following setting parameters https://www.swpc.noaa.gov/phenomena/f107-cm-radio-emissions
  double manual_daily_f107_;    //!< Manual daily f10.7 value
  double manual_average_f107_;  //!< Manual 3-month averaged f10.7 value
  double manual_ap_;            //!< Manual ap value Ref: http://wdc.kugi.kyoto-u.ac.jp/kp/kpexp-j.html

  //  double rw_stepwidth_;
  //  double rw_stddev_;
  //  double rw_limit_;

  /**
   * @fn CalcStandard
   * @brief Calculate atmospheric density with simplest method
   * @param [in] altitude_m: Altitude of spacecraft [m]
   * @return Atmospheric density [kg/m^3]
   */
  double CalcStandard(double altitude_m);
  /**
   * @fn GetSpaceWeatherTable
   * @param [in] decyear: Decimal year of simulation start [year]
   * @param [in] endsec: End time of simulation [sec]
   * @return Size of table
   */
  int GetSpaceWeatherTable(double decyear, double endsec);

  /**
   * @fn AddNoise
   * @brief Add atmospheric density noise
   * @param [in] rho: True atmospheric density [kg/m^3]
   * @return Atmospheric density with noise [kg/m^3]
   */
  double AddNoise(double rho);
};

#endif  // S2E_ENVIRONMENT_LOCAL_ATMOSPHERE_H_
