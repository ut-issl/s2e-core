/**
 * @file geopotential.hpp
 * @brief Class to calculate the high-order earth gravity acceleration
 */

#ifndef S2E_DISTURBANCES_GEOPOTENTIAL_HPP_
#define S2E_DISTURBANCES_GEOPOTENTIAL_HPP_

#include <string>

#include "../library/gravity/gravity_potential.hpp"
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

  /**
   * @fn Geopotential
   * @brief Copy Constructor
   */
  Geopotential(const Geopotential &obj) : Disturbance(obj) {
    geopotential_ = obj.geopotential_;
    degree_ = obj.degree_;
    c_ = obj.c_;
    s_ = obj.s_;
  }

  ~Geopotential() {}

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

  libra::Vector<3> CalcAcceleration_ecef_m_s2(const libra::Vector<3> position_ecef_m) {
    return geopotential_.CalcAcceleration_xcxf_m_s2(position_ecef_m);
  }

 private:
  GravityPotential geopotential_;
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

/**
 * @fn InitGeopotential
 * @brief Initialize Geopotential class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 */
Geopotential InitGeopotential(const std::string initialize_file_path);

#endif  // S2E_DISTURBANCES_GEOPOTENTIAL_HPP_
