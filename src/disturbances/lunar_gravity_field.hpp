/**
 * @file lunar_gravity_field.hpp
 * @brief Class to calculate the high-order lunar gravity acceleration
 */

#ifndef S2E_DISTURBANCES_LUNAR_GRAVITY_FIELD_HPP_
#define S2E_DISTURBANCES_LUNAR_GRAVITY_FIELD_HPP_

#include <string>

#include "../library/gravity/gravity_potential.hpp"
#include "../library/math/vector.hpp"
#include "disturbance.hpp"
/**
 * @class LunarGravityField
 * @brief Class to calculate the high-order earth gravity acceleration
 */
class LunarGravityField : public Disturbance {
 public:
  /**
   * @fn Geopotential
   * @brief Constructor
   * @param [in] degree: Maximum degree setting to calculate the geo-potential
   * @param [in] file_path: EGM96 coefficients file path
   * @param [in] is_calculation_enabled: Calculation flag
   */
  LunarGravityField(const int degree, const std::string file_path, const bool is_calculation_enabled = true);

  /**
   * @fn LunarGravityField
   * @brief Copy Constructor
   */
  LunarGravityField(const LunarGravityField &obj) : Disturbance(obj) {
    lunar_potential_ = obj.lunar_potential_;
    reference_radius_km_ = obj.reference_radius_km_;
    gravity_constants_km3_s2_ = obj.gravity_constants_km3_s2_;
    degree_ = obj.degree_;
    c_ = obj.c_;
    s_ = obj.s_;
  }

  ~LunarGravityField() {}

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
  GravityPotential lunar_potential_;
  double reference_radius_km_;
  double gravity_constants_km3_s2_;
  size_t degree_;                       //!< Maximum degree setting to calculate the geo-potential
  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients
  Vector<3> acceleration_mcmf_m_s2_;    //!< Calculated acceleration in the MCMF(Moon Centered Moon Fixed) frame [m/s2]

  // debug
  libra::Vector<3> debug_pos_mcmf_m_;  //!< Spacecraft position in MCMF frame [m]
  double time_ms_ = 0.0;               //!< Calculation time [ms]

  /**
   * @fn ReadCoefficientsGrgm1200a
   * @brief Read the lunar gravity field coefficients for the GRGM1200A model
   * @param [in] file_name: Coefficient file name
   */
  bool ReadCoefficientsGrgm1200a(std::string file_name);
};

#endif  // S2E_DISTURBANCES_LUNAR_GRAVITY_FIELD_HPP_
