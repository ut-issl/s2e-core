/**
 * @file Structure.h
 * @brief Definition of spacecraft structure
 */

#pragma once
#include <Simulation/SimulationConfig.h>

#include <vector>

#include "KinematicsParams.h"
#include "RMMParams.h"
#include "Surface.h"
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
  Structure(SimulationConfig* sim_config, const int sat_id);
  /**
   * @fn ~Structure
   * @brief Destructor
   */
  ~Structure();
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize(SimulationConfig* sim_config, const int sat_id);

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
  inline const KinematicsParams& GetKinematicsParams() const { return *kinnematics_params_; }
  /**
   * @fn GetRMMParams
   * @brief Return Residual Magnetic Moment information
   */
  inline const RMMParams& GetRMMParams() const { return *rmm_params_; }

 private:
  KinematicsParams* kinnematics_params_;  //!< Kinematics parameters
  vector<Surface> surfaces_;              //!< Surface information
  RMMParams* rmm_params_;                 //!< Residual Magnetic Moment
};
