/**
 * @file solar_radiation_pressure_disturbance.hpp
 * @brief Class to calculate the solar radiation pressure disturbance force and torque
 */

#ifndef S2E_DISTURBANCES_SOLAR_RADIATION_PRESSURE_DISTURBANCE_H_
#define S2E_DISTURBANCES_SOLAR_RADIATION_PRESSURE_DISTURBANCE_H_

#include <Library/utils/Macros.hpp>
#include <string>

#include "../Interface/LogOutput/ILoggable.h"
#include "../Library/math/Vector.hpp"
#include "surface_force.hpp"
using libra::Vector;

/**
 * @class SolarRadiation
 * @brief Class to calculate the solar radiation pressure disturbance force and torque
 */
class SolarRadiation : public SurfaceForce {
 public:
  /**
   * @fn SolarRadiation
   * @brief Constructor
   */
  SolarRadiation(const vector<Surface>& surfaces, const Vector<3>& cg_b);

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
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
   * @fn CalcCoef
   * @brief Override CalcCoef function of SurfaceForce
   * @param [in] input_b: Direction vector of the sun at the body frame
   * @param [in] item: Solar pressure [N/m^2]
   */
  virtual void CalcCoef(Vector<3>& input_b, double item);
};

#endif  // S2E_DISTURBANCES_SOLAR_RADIATION_PRESSURE_DISTURBANCE_H_
