/**
 * @file atmosphere.hpp
 * @brief Class to calculate earth's atmospheric density
 */
#ifndef S2E_ENVIRONMENT_LOCAL_ATMOSPHERE_HPP_
#define S2E_ENVIRONMENT_LOCAL_ATMOSPHERE_HPP_

#include <string>
#include <vector>

#include "dynamics/orbit/orbit.hpp"
#include "environment/global/simulation_time.hpp"
#include "environment/local/local_celestial_information.hpp"
#include "logger/loggable.hpp"
#include "math_physics/atmosphere/wrapper_nrlmsise00.hpp"
#include "math_physics/math/vector.hpp"

/**
 * @class Atmosphere
 * @brief Class to calculate earth's atmospheric density
 */
class Atmosphere : public ILoggable {
 public:
  /**
   * @fn Atmosphere
   * @brief Constructor
   * @param [in] model: Atmospheric density model name
   * @param [in] space_weather_file_name: Path and name of space weather file
   * @param [in] gauss_standard_deviation_rate: Standard deviation of density noise (defined as percentage)
   * @param [in] is_manual_param: Flag to use manual parameters
   * @param [in] manual_f107: Manual value of daily F10.7
   * @param [in] manual_f107a: Manual value of averaged F10.7 (3-month averaged value)
   * @param [in] manual_ap: Manual value of ap value
   * @param [in] local_celestial_information: Local Celestial information
   * @param [in] simulation_time: Simulation Time information
   */
  Atmosphere(const std::string model, const std::string space_weather_file_name, const double gauss_standard_deviation_rate,
             const bool is_manual_param, const double manual_f107, const double manual_f107a, const double manual_ap,
             const LocalCelestialInformation* local_celestial_information, const SimulationTime* simulation_time);
  /**
   * @fn ~Atmosphere
   * @brief Destructor
   */
  virtual ~Atmosphere() {}
  /**
   * @fn CalcAirDensity
   * @brief Calculate atmospheric density
   * @param [in] decimal_year: Decimal year of simulation start [year]
   * @param [in] position: Position of target point to calculate the air density
   * @return Atmospheric density [kg/m^3]
   */
  double CalcAirDensity_kg_m3(const double decimal_year, const Orbit& orbit);
  /**
   * @fn GetAirDensity
   * @brief Return Atmospheric density [kg/m^3]
   */
  inline double GetAirDensity_kg_m3() const { return air_density_kg_m3_; }
  /**
   * @fn SetCalcFlag
   * @brief Set calculation flag (true: Enable, false: Disable)
   */
  inline void SetCalcFlag(const bool is_calc_enabled) { is_calc_enabled_ = is_calc_enabled; }

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
  // General information
  bool is_calc_enabled_ = true;  //!< Calculation enable flag
  std::string model_;            //!< Atmospheric density model name
  double air_density_kg_m3_;     //!< Atmospheric density [kg/m^3]

  // NRLMSISE-00 model information
  std::vector<nrlmsise_table> space_weather_table_;  //!< Space weather table
  bool is_manual_param_used_;                        //!< Flag to use manual parameters
  // Reference of the following setting parameters https://www.swpc.noaa.gov/phenomena/f107-cm-radio-emissions
  double manual_daily_f107_;    //!< Manual daily f10.7 value
  double manual_average_f107_;  //!< Manual 3-month averaged f10.7 value
  double manual_ap_;            //!< Manual ap value Ref: http://wdc.kugi.kyoto-u.ac.jp/kp/kpexp-j.html

  // Noise Information
  double gauss_standard_deviation_rate_;  //!< Standard deviation of density noise (defined as percentage)
  // TODO: Add random walk noise
  //  double rw_stepwidth_;
  //  double rw_stddev_;
  //  double rw_limit_;

  // References
  const LocalCelestialInformation* local_celestial_information_;  //!< Local celestial information

  // Functions
  /**
   * @fn AddNoise
   * @brief Add atmospheric density noise
   * @param [in] rho_kg_m3: True atmospheric density [kg/m^3]
   * @return Atmospheric density with noise [kg/m^3]
   */
  double AddNoise(const double rho_kg_m3);
};

/**
 * @fn InitAtmosphere
 * @brief Initialize atmospheric density of the earth
 * @param [in] initialize_file_path: Path to initialize file
 * @param [in] local_celestial_information: Local celestial information
 */
Atmosphere InitAtmosphere(const std::string initialize_file_path, const LocalCelestialInformation* local_celestial_information,
                          const SimulationTime* simulation_time);

#endif  // S2E_ENVIRONMENT_LOCAL_ATMOSPHERE_HPP_
