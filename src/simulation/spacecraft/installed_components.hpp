/**
 * @file installed_components.hpp
 * @brief Definition of InstalledComponents class
 */

#ifndef S2E_SIMULATION_SPACECRAFT_INSTALLED_COMPONENTS_HPP_
#define S2E_SIMULATION_SPACECRAFT_INSTALLED_COMPONENTS_HPP_

#include <logger/logger.hpp>
#include <math_physics/math/vector.hpp>

/**
 * @class InstalledComponents
 * @brief Base class to express components list installed on a spacecraft
 */
class InstalledComponents {
 public:
  /**
   * @fn ~InstalledComponents
   * @brief Destructor
   */
  virtual ~InstalledComponents() {}

  /**
   * @fn GenerateForce_b_N
   * @brief Return force generated by components in unit Newton in body fixed frame
   * @details Users need to override this function to add force generated by components
   */
  virtual math::Vector<3> GenerateForce_b_N();

  /**
   * @fn GenerateTorque_b_Nm
   * @brief Return torque generated by components in unit Newton-meter in body fixed frame
   * @details Users need to override this function to add torque generated by components
   */
  virtual math::Vector<3> GenerateTorque_b_Nm();

  /**
   * @fn ComponentInterference
   * @brief Handle component interference effect
   */
  virtual void ComponentInterference() {}

  /**
   * @fn LogSetup
   * @brief Setup the logger for components
   * @details Users need to override this function to add logger for components
   */
  virtual void LogSetup(Logger& logger);
};

#endif  // S2E_SIMULATION_SPACECRAFT_INSTALLED_COMPONENTS_HPP_
