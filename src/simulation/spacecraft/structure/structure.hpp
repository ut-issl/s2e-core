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
using std::vector;

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
  Structure(SimulationConfig* simulation_configuration, const int spacecraft_id);
  /**
   * @fn ~Structure
   * @brief Destructor
   */
  ~Structure();
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize(SimulationConfig* simulation_configuration, const int spacecraft_id);

  // Getter
  /**
   * @fn GetSurfaces
   * @brief Return surface information
   */
  inline const vector<Surface>& GetSurfaces() const { return surfaces_; }
  /**
   * @fn GetKinematicsParams
   * @brief Return kinematics information
   */
  inline const KinematicsParameters& GetKinematicsParams() const { return *kinnematics_params_; }
  /**
   * @fn GetRMMParams
   * @brief Return Residual Magnetic Moment information
   */
  inline const RMMParams& GetRMMParams() const { return *rmm_params_; }

  /**
   * @fn GetToSetSurfaces
   * @brief Return surface information
   */
  inline vector<Surface>& GetToSetSurfaces() { return surfaces_; }
  /**
   * @fn GetToSetKinematicsParams
   * @brief Return kinematics information
   */
  inline KinematicsParameters& GetToSetKinematicsParams() { return *kinnematics_params_; }
  /**
   * @fn GetToSetRMMParams
   * @brief Return Residual Magnetic Moment information
   */
  inline RMMParams& GetToSetRMMParams() { return *rmm_params_; }

 private:
  KinematicsParameters* kinnematics_params_;  //!< Kinematics parameters
  vector<Surface> surfaces_;                  //!< Surface information
  RMMParams* rmm_params_;                     //!< Residual Magnetic Moment
};

#endif  // S2E_SIMULATION_SPACECRAFT_STRUCTURE_STRUCTURE_HPP_
