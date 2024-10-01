/**
 * @file solar_radiation_pressure_disturbance.hpp
 * @brief Class to calculate the solar radiation pressure disturbance force and torque
 */

#ifndef S2E_DISTURBANCES_SOLAR_RADIATION_PRESSURE_DISTURBANCE_HPP_
#define S2E_DISTURBANCES_SOLAR_RADIATION_PRESSURE_DISTURBANCE_HPP_

#include <utilities/macros.hpp>

#include "../logger/loggable.hpp"
#include "../math_physics/math/vector.hpp"
#include "surface_force.hpp"

namespace s2e::disturbances {

/**
 * @class SolarRadiationPressureDisturbance
 * @brief Class to calculate the solar radiation pressure disturbance force and torque
 */
class SolarRadiationPressureDisturbance : public SurfaceForce {
 public:
  /**
   * @fn SolarRadiationPressureDisturbance
   * @brief Constructor
   * @param [in] surfaces: Surface information of the spacecraft
   * @param [in] center_of_gravity_b_m: Center of gravity position at the body frame [m]
   * @param [in] is_calculation_enabled: Calculation flag
   */
  SolarRadiationPressureDisturbance(const std::vector<spacecraft::Surface>& surfaces, const math::Vector<3>& center_of_gravity_b_m,
                                    const bool is_calculation_enabled = true);

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
   * @param [in] local_environment: Local environment information
   * @param [in] dynamics: dynamics::Dynamics information
   */
  virtual void Update(const environment::LocalEnvironment& local_environment, const dynamics::Dynamics& dynamics);

  // Override logger::ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of logger::ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of logger::ILoggable
   */
  virtual std::string GetLogValue() const;

 private:
  /**
   * @fn CalcCoefficients
   * @brief Override CalcCoefficients function of SurfaceForce
   * @param [in] input_direction_b: Direction vector of the sun at the body frame
   * @param [in] item: Solar pressure [N/m^2]
   */
  void CalcCoefficients(const math::Vector<3>& input_direction_b, const double item);
};

/**
 * @fn InitSolarRadiationPressureDisturbance
 * @brief Initialize SolarRadiationPressureDisturbance class
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] surfaces: surface information of the spacecraft
 * @param [in] center_of_gravity_b_m: Center of gravity position vector at body frame [m]
 */
SolarRadiationPressureDisturbance InitSolarRadiationPressureDisturbance(const std::string initialize_file_path,
                                                                        const std::vector<spacecraft::Surface>& surfaces,
                                                                        const math::Vector<3>& center_of_gravity_b_m);

}  // namespace s2e::disturbances

#endif  // S2E_DISTURBANCES_SOLAR_RADIATION_PRESSURE_DISTURBANCE_HPP_
