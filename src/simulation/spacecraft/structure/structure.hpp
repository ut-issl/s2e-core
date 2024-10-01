/**
 * @file structure.hpp
 * @brief Definition of spacecraft structure
 */

#ifndef S2E_SIMULATION_SPACECRAFT_STRUCTURE_STRUCTURE_HPP_
#define S2E_SIMULATION_SPACECRAFT_STRUCTURE_STRUCTURE_HPP_

#include <simulation/simulation_configuration.hpp>
#include <vector>

#include "kinematics_parameters.hpp"
#include "residual_magnetic_moment.hpp"
#include "surface.hpp"

namespace s2e::simulation {

/**
 * @class Structure
 * @brief Class for spacecraft structure information
 */
class Structure {
 public:
  /**
   * @fn Structure
   * @brief Constructor
   */
  Structure(const SimulationConfiguration* simulation_configuration, const int spacecraft_id);
  /**
   * @fn ~Structure
   * @brief Destructor
   */
  ~Structure();
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize(const SimulationConfiguration* simulation_configuration, const int spacecraft_id);

  // Getter
  /**
   * @fn GetSurfaces
   * @brief Return surface information
   */
  inline const std::vector<simulation::Surface>& GetSurfaces() const { return surfaces_; }
  /**
   * @fn GetKinematicsParameters
   * @brief Return kinematics information
   */
  inline const KinematicsParameters& GetKinematicsParameters() const { return *kinematics_parameters_; }
  /**
   * @fn GetResidualMagneticMoment
   * @brief Return Residual Magnetic Moment information
   */
  inline const ResidualMagneticMoment& GetResidualMagneticMoment() const { return *residual_magnetic_moment_; }

  /**
   * @fn GetToSetSurfaces
   * @brief Return surface information
   */
  inline std::vector<simulation::Surface>& GetToSetSurfaces() { return surfaces_; }
  /**
   * @fn GetToSetKinematicsParameters
   * @brief Return kinematics information
   */
  inline KinematicsParameters& GetToSetKinematicsParameters() { return *kinematics_parameters_; }
  /**
   * @fn GetToSetResidualMagneticMoment
   * @brief Return Residual Magnetic Moment information
   */
  inline ResidualMagneticMoment& GetToSetResidualMagneticMoment() { return *residual_magnetic_moment_; }

 private:
  KinematicsParameters* kinematics_parameters_;       //!< Kinematics parameters
  std::vector<simulation::Surface> surfaces_;         //!< Surface information
  ResidualMagneticMoment* residual_magnetic_moment_;  //!< Residual Magnetic Moment
};

}  // namespace s2e::simulation

#endif  // S2E_SIMULATION_SPACECRAFT_STRUCTURE_STRUCTURE_HPP_
