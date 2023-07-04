/**
 * @file geopotential.hpp
 * @brief Class to calculate the high-order earth gravity acceleration
 */

#ifndef S2E_DISTURBANCES_GEOPOTENTIAL_HPP_
#define S2E_DISTURBANCES_GEOPOTENTIAL_HPP_

#include <string>

#include "../library/gravity_potential/gravity_potential.hpp"
#include "../library/math/vector.hpp"
#include "disturbance.hpp"
/**
 * @class Geopotential
 * @brief Class to calculate the high-order earth gravity acceleration
 */
class Geopotential : public Disturbance {
 public:
  /**
   * @fn Geopotential
   * @brief Constructor
   * @param [in] degree: Maximum degree setting to calculate the geo-potential
   * @param [in] file_path: EGM96 coefficients file path
   * @param [in] is_calculation_enabled: Calculation flag
   */
  Geopotential(const int degree, const std::string file_path, const bool is_calculation_enabled = true);

  ~Geopotential() { delete geopotential_; }

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
   * @param [in] local_environment: Local environment information
   * @param [in] dynamics: Dynamics information
   */
  virtual void Update(const LocalEnvironment &local_environment, const Dynamics &dynamics);

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
  GravityPotential *geopotential_;
  size_t degree_;                       //!< Maximum degree setting to calculate the geo-potential
  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients
  Vector<3> acceleration_ecef_m_s2_;    //!< Calculated acceleration in the ECEF frame [m/s2]

  // debug
  libra::Vector<3> debug_pos_ecef_m_;  //!< Spacecraft position in ECEF frame [m]
  double time_ms_ = 0.0;               //!< Calculation time [ms]

  /**
   * @fn ReadCoefficientsEgm96
   * @brief Read the geo-potential coefficients for the EGM96 model
   * @param [in] file_name: Coefficient file name
   */
  bool ReadCoefficientsEgm96(std::string file_name);
};

#endif  // S2E_DISTURBANCES_GEOPOTENTIAL_HPP_
