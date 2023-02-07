/**
 * @file SampleGS.h
 * @brief An example of user defined ground station class
 */

#pragma once

#include <Component/CommGS/GScalculator.h>
#include <Dynamics/Dynamics.h>
#include <Environment/Global/GlobalEnvironment.h>

#include <Component/CommGS/Antenna.hpp>

#include "../../spacecraft/sample_spacecraft/sample_spacecraft.hpp"
#include "../GroundStation.h"

class SampleGSComponents;

/**
 * @class SampleGS
 * @brief An example of user defined ground station class
 */
class SampleGS : public GroundStation {
 public:
  /**
   * @fn SampleGS
   * @brief Constructor
   */
  SampleGS(SimulationConfig* config, int gs_id);
  /**
   * @fn ~SampleGS
   * @brief Destructor
   */
  ~SampleGS();

  /**
   * @fn Initialize
   * @brief Override function of Initialize in GroundStation class
   */
  virtual void Initialize(SimulationConfig* config);
  /**
   * @fn LogSetup
   * @brief Override function of LogSetup in GroundStation class
   */
  virtual void LogSetup(Logger& logger);
  /**
   * @fn Update
   * @brief Override function of Update in GroundStation class
   */
  virtual void Update(const CelestialRotation& celes_rotation, const SampleSat& spacecraft);

 private:
  SampleGSComponents* components_;  //!< Ground station related components
};
