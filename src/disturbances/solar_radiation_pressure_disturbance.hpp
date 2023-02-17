/**
 * @file solar_radiation_pressure_disturbance.hpp
 * @brief Class to calculate the solar radiation pressure disturbance force and torque
 */

#ifndef S2E_DISTURBANCES_SOLAR_RADIATION_PRESSURE_DISTURBANCE_HPP_
#define S2E_DISTURBANCES_SOLAR_RADIATION_PRESSURE_DISTURBANCE_HPP_

#include <library/utilities/macros.hpp>

#include "../library/logger/loggable.hpp"
#include "../library/math/vector.hpp"
#include "surface_force.hpp"

/**
 * @class SolarRadiation
 * @brief Class to calculate the solar radiation pressure disturbance force and torque
 */
class SolarRadiation : public SurfaceForce {
 public:
  /**
   * @fn SolarRadiation
   * @brief Constructor
   * @param [in] surfaces: Surface information of the spacecraft
   * @param [in] center_of_gravity_b_m: Center of gravity position at the body frame [m]
   * @param [in] is_calculation_enabled: Calculation flag
   */
  SolarRadiation(const std::vector<Surface>& surfaces, const libra::Vector<3>& center_of_gravity_b_m, const bool is_calculation_enabled = true);

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
   * @param [in] local_environment: Local environment information
   * @param [in] dynamics: Dynamics information
   */
  virtual void Update(const LocalEnvironment& local_env, const Dynamics& dynamics);

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
  /**
   * @fn CalcCoefficients
   * @brief Override CalcCoefficients function of SurfaceForce
   * @param [in] input_direction_b: Direction vector of the sun at the body frame
   * @param [in] item: Solar pressure [N/m^2]
   */
  void CalcCoefficients(const libra::Vector<3>& input_direction_b, const double item);
};

#endif  // S2E_DISTURBANCES_SOLAR_RADIATION_PRESSURE_DISTURBANCE_HPP_
