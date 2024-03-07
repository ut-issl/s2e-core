/**
 * @file third_body_gravity.hpp
 * @brief Class to calculate third body gravity disturbance
 */

#ifndef S2E_DISTURBANCES_THIRD_BODY_GRAVITY_HPP_
#define S2E_DISTURBANCES_THIRD_BODY_GRAVITY_HPP_

#include <cassert>
#include <set>
#include <string>

#include "../math_physics/math/vector.hpp"
#include "../logger/loggable.hpp"
#include "disturbance.hpp"

/**
 * @class ThirdBodyGravity
 * @brief Class to calculate third body gravity disturbance
 */
class ThirdBodyGravity : public Disturbance {
 public:
  /**
   * @fn ThirdBodyGravity
   * @brief Constructor
   * @param [in] third_body_list: List of calculation target bodies
   * @param [in] is_calculation_enabled: Calculation flag
   */
  ThirdBodyGravity(const std::set<std::string> third_body_list, const bool is_calculation_enabled = true);
  /**
   * @fn ~ThirdBodyGravity
   * @brief Destructor
   */
  ~ThirdBodyGravity();

  /**
   * @fn Update
   * @brief Update third body disturbance
   * @param [in] local_environment: Local environment information
   * @param [in] dynamics: Dynamics information
   */
  virtual void Update(const LocalEnvironment& local_environment, const Dynamics& dynamics);

 private:
  std::set<std::string> third_body_list_;                 //!< List of celestial bodies to calculate the third body disturbances
  libra::Vector<3> third_body_acceleration_i_m_s2_{0.0};  //!< Calculated third body disturbance acceleration in the inertial frame [m/s2]

  // Override classes for ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override function of GetLogHeader
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override function of GetLogValue
   */
  virtual std::string GetLogValue() const;

  /**
   * @fn CalcAcceleration_i_m_s2
   * @brief Calculate and return the third body disturbance acceleration
   * @param [in] s: Position vector of the third celestial body from the origin in the inertial frame in unit [m]
   * @param [in] sr: Position vector of the third celestial body from the spacecraft in the inertial frame in unit [m]
   * @param [in] GM: The gravitational constants of the third celestial body [m3/s2]
   * @return Third body disturbance acceleration in the inertial frame in unit [m/s2]
   */
  libra::Vector<3> CalcAcceleration_i_m_s2(const libra::Vector<3> s, const libra::Vector<3> sr, const double gravity_constant_m_s2);
};

/**
 * @fn InitThirdBodyGravity
 * @brief Initialize ThirdBodyGravity class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] ini_path_celes: Initialize file path for the celestial information
 */
ThirdBodyGravity InitThirdBodyGravity(const std::string initialize_file_path, const std::string ini_path_celes);

#endif  // S2E_DISTURBANCES_THIRD_BODY_GRAVITY_HPP_
